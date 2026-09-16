# Synchronization contract

## Synchronous mode (default)

One bridge owns and ticks a CARLA world. It enables fixed-delta synchronous mode
on both the world and Traffic Manager, configures before the repeatable episode
reload, and seeds NPC spawning/Traffic Manager. These measures improve
repeatability; they are not a claim of bit-identical runs across builds/hardware.

`control_ego_state()` applies a command, ticks once, and waits for the exact frame
number returned by `world.tick()`. `step()` is the corresponding direct-script
API; never call both for the same intended step. Getters do not tick.

The LiDAR rotation rate is `1 / fixed_delta_seconds`, so each callback covers one
full rotation at one simulated instant. RGB `sensor_tick` is zero (every simulation
step). The camera and LiDAR callbacks attach their CARLA frame and timestamp.

Callbacks write to bounded frame-indexed queues under one `Condition`. A capture
is complete only when both devices report the same frame and matching timestamps.
Readers take the tick lock, so they cannot see a partially delivered next tick.
GPU camera delay is handled by waiting on the condition, not polling `time.time()`.

On timeout, the bridge raises `TimeoutError` and latches a fault. It does not return
the old capture as if the new tick succeeded, and refuses another tick until reset.
CARLA may already have advanced when a sensor fails; resuming without reset would
let the core's elapsed-time counter drift. `close()` wakes blocked waiters.

## Clock policy

Sync mode requires `c40_pace_sim=true` and a finite, positive `c40_sim_dt`. The
bridge reads `c40_sim_dt` at construction and uses it as CARLA's
`fixed_delta_seconds`; there is no separate plugin or constructor step-size
setting. This applies to both factory-created bridges and explicit
`sync_mode=True`. Every explicit `dt`
must match that fixed step. Recreate the bridge after changing `c40_sim_dt`.
Validation happens before simulation mutation.

This is a bridge-side guard around a preexisting core limitation: the core counts
requested dt, not the simulator's actual clock. It is not a general clock-interface
redesign. Choose `SyncExecuter` for sequential pose/perception/planning execution.
`AsyncThreadedExecuter` can read coherent sensor pairs, but its shared
`PerceptionModel` and independently fetched ground truth are not frame-atomic.

Direct scripts may specify `sync_mode` explicitly and drive `step(dt)` themselves,
using AVLite's execution settings to configure the synchronous step duration.
An omitted `dt` uses the configured fixed step. The method returns only after that
frame's pair is ready. Keep exactly one ticking client per CARLA world.

## Asynchronous mode

The server advances independently. Callbacks are still matched by exact CARLA
frame and timestamp; an early camera or LiDAR waits in the bounded queue.
`get_sensor_frame()` returns the newest completed pair, not the newest sample of each
modality. Old/out-of-order frames cannot move the published frame backward.
If no completed pair arrives within `max_sensor_age` wall seconds, reads raise
`TimeoutError` instead of silently reusing a stale pair.

Async LiDAR is deliberately one callback/instant, which may be a partial arc.
For example, a 10 Hz scanner at 20 simulation FPS covers about 180 degrees per
callback. There is no concatenation of different instants disguised as a
single-time synchronized cloud. Full-sweep fusion would need scan intervals,
per-point timing and an explicit motion-compensation policy; it is not implemented.

The initial capture before data arrives has known sensor metadata and `None`
readings. Consumers must handle that normal startup state.

## Frames and projection

LiDAR coordinates remain sensor-local with CARLA Y reflected to AVLite Y-left.
The sensor's static mount is separate. Camera mounts include optical-axis
conversion; neither mount depends on the changing ego pose.

`CarlaSensorFrame.base_to_map` stores the actual acquisition pose reconstructed
from the LiDAR measurement transform and its mount. `world_to_camera(frame)` uses
that stored pose, including roll/pitch, so an old image's projection does not
change when the car moves. The frame carries the actual per-device timestamps;
the assembly stamp remains unspecified rather than hiding stale data with `max()`.

## Reset, teleport and shutdown

Reset/teleport invalidate the callback generation before destroying sensor actors.
Late callbacks from those actors cannot populate a new episode, even when CARLA
frame numbers restart. Sensor queues, faults and published captures are cleared.
Reset preserves/reapplies world and Traffic Manager timing and restores NPCs.

The spectator thread has a stop event and is joined on close. `close()` destroys
owned actors and restores the pre-bridge world settings. Starting another bridge
for the same server closes the previous owner, including across `importlib.reload()`.
An `atexit` hook closes active bridge owners when Python exits normally. A hard
kill cannot run cleanup; CARLA may need its synchronous mode disabled externally.

NPC spawning never replaces the ego reference and accepts AVLite's `global_plan`
keyword. Ego spawning/teleport use centralized handedness conversion, including
elevation; a failed spawn raises rather than continuing with `vehicle=None`.

## References

- [Synchrony and time-step](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)
- [Sensors reference](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)
- [Python API](https://carla.readthedocs.io/en/latest/python_api/#carla.Client)
