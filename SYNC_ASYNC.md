# Sync vs Async mode

`Carla4Bridge` can run CARLA in two modes, controlled by `sync_mode` (default
`False`, i.e. async). This is about how **the CARLA server itself** advances —
a different, orthogonal thing from AVLite's own `SyncExecuter` /
`AsyncThreadedExecuter` choice (`c40_executer_type`), which is about how the
**stack** (perception/planning/control) is scheduled. You can pair either
executer with either bridge mode; see "Which one should I use" below for the
one combination worth avoiding.

See `FIXES.md` (Fix 2) for the implementation history and the specific bugs
this was built to fix.

## Async mode (default)

CARLA free-runs in real time — nothing in the bridge ticks it. Each attached
sensor delivers data on its own schedule, on CARLA's own callback threads,
completely independent of the other:

- LiDAR completes a full 360° sweep every `1 / LIDAR_ROTATION_FREQUENCY`
  seconds (10Hz by default → every ~100ms).
- The RGB camera fires every rendered frame (`sensor_tick=0.0`) — however fast
  the server happens to be rendering, which is usually faster than the LiDAR.

`_on_lidar`/`_on_rgb` each just overwrite their own buffer as data arrives,
stamped with `(frame, timestamp)` from that measurement. Nothing coordinates
the two callbacks with each other.

**Async only *approximates* synchronization.** `get_sensor_frame()` reads
whatever's currently sitting in each buffer at the moment it's called — which
could be a fresh RGB frame paired with a LiDAR sweep that's several sweeps
old, or vice versa. It compares the two buffers' frame numbers and logs a
warning if they diverge past `SENSOR_SKEW_TOLERANCE_FRAMES`, but it does not
block, retry, or drop data to fix this — there's no way to force two
independently-clocked sensors back in step without CARLA ticking in lockstep,
which is exactly what sync mode is for. Async mode's guarantee is honesty
(you'll know when a pair is misaligned), not correctness.

## Sync mode (`sync_mode=True`)

CARLA only advances one fixed-size step (`fixed_delta_seconds`, default
`0.05`s) per explicit `world.tick()` call, issued by this bridge — the server
otherwise sits frozen. This bridge ticks it in exactly one place:
`control_ego_state()`, right after applying the vehicle control for that
cycle (also reachable directly via the `step()` hook for callers that don't
go through `control_ego_state()`, e.g. a script that only spawns agents).

**Sync mode is always synced by construction, not by checking.** After
`world.tick()`, `__tick()` calls `__wait_for_sensor_frame()`, which blocks
(bounded by a short timeout) until *both* the LiDAR and RGB buffers are
stamped with that tick's frame number before returning. So by the time
anything calls `get_sensor_frame()` afterward, the pair is already guaranteed
aligned — the skew check still runs, but it's a trip-wire for "something's
actually broken" (e.g. a sensor stopped delivering), not a normal-path
warning like in async mode.

Two other things are set up specifically for sync mode:

- **LiDAR `rotation_frequency`** is set to `1 / fixed_delta_seconds` instead
  of the fixed 10Hz default, so a full 360° sweep completes in *exactly* one
  tick. At the default 10Hz, a 20Hz tick rate (`fixed_delta_seconds=0.05`)
  would otherwise only deliver half a rotation per tick.
- **The traffic manager** (which drives NPC autopilot from
  `spawn_npc_vehicles()`) is put in sync mode too, via
  `traffic_manager.set_synchronous_mode(True)` — it has its own sync flag,
  independent of the world's, and NPCs would desync or stall without it.

## Which one should I use

- **Sync mode** — reproducible/deterministic runs: offline evaluation,
  logging, dataset generation, anything comparing behavior across repeated
  runs, and anything that projects LiDAR points into the camera image plane
  (misaligned frames there produce a silently wrong projection whenever
  anything is moving).
- **Async mode** — real-time/interactive use: visualization, manual driving,
  live demos, anything where wall-clock pacing matters more than frame-exact
  alignment and a little skew is tolerable.
- **Avoid**: sync mode paired with `AsyncThreadedExecuter`. It still runs —
  `control_ego_state()` is called from a single dedicated controller thread,
  so ticking stays well-defined — but perception/localization read sensor
  data on their own threads, on their own timers, uncoordinated with tick
  boundaries. You lose most of sync mode's determinism benefit even though
  nothing breaks. Pair sync mode with `SyncExecuter` if determinism is the
  actual goal.

### Enabling it

`executor_factory()` (AVLite core) only forwards
`ego_state`/`pm`/`reference_point`/`map` to bridge constructors, so going
through the normal AVLite config path, `sync_mode`/`fixed_delta_seconds`
aren't reachable as constructor kwargs — set them in this plugin's own
`PluginSettings` instead, at `~/.config/avlite/plugin_avlite-bridge-carla.yaml`:

```yaml
sync_mode: true
fixed_delta_seconds: 0.05
```

Instantiating `Carla4Bridge(...)` directly (scripts, tests) can pass
`sync_mode`/`fixed_delta_seconds` as constructor kwargs instead, which take
priority over `PluginSettings` when given.

## Choosing `SENSOR_SKEW_TOLERANCE_FRAMES`

This is a count of CARLA simulation steps apart, not a fixed amount of time —
`measurement.frame`/`image.frame` increments by exactly 1 each time the
server advances, whether that's a `tick()` in sync mode or one of the
server's own internal steps in async mode. Whether "N frames" is a precise or
approximate time bound depends entirely on which mode you're in:

- **In sync mode**, each step is exactly `fixed_delta_seconds` long, always —
  so `frame_skew * fixed_delta_seconds` is an *exact* time delta. The
  tolerance barely matters here in practice, since `__wait_for_sensor_frame()`
  keeps skew at 0 by construction; it only fires if something's actually
  wrong (e.g. a sensor actor stalled or was destroyed mid-run).
- **In async mode**, each internal step's wall-clock duration is elastic — it
  depends on whatever FPS the server happens to be hitting at that moment
  (render load, GPU stalls, other clients). "2 frames apart" might be ~30ms
  during a smooth stretch or much more during a hitch. Treat the tolerance as
  a rough heuristic here, not a precise time bound — the frame-count is a
  proxy for time, not time itself, only in this mode.

To pick a value for async mode, reason from how much positional error your
downstream consumer can tolerate, not from the frame count directly. Rough
back-of-envelope: at a server running ~30 FPS and a vehicle moving ~10 m/s,
one frame ≈ 33ms ≈ 33cm of travel. If you're projecting LiDAR points into the
camera image and care about centimetre-level alignment, even 1-2
frames of gap can visibly offset points; loosen the tolerance if your
consumer only needs coarse alignment and you'd rather not have the log noisy.
The current default (`2`) is a starting point, not a validated number for any
particular use case — it only affects logging (`get_sensor_frame()` always
returns whatever's buffered regardless of skew today), so tightening it makes
the warning fire more often without changing behavior otherwise.
