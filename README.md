# AVLite CARLA bridge

`Carla4Bridge` connects AVLite to CARLA with synchronized RGB camera and 3D
LiDAR captures. It supports synchronous fixed-step execution, asynchronous
sensor delivery, CARLA ground-truth detection/tracking/localization, NPC
spawning, and capture-time sensor geometry.

This plugin targets the named multi-sensor `SensorFrame` API on AVLite's 0.6.4 or newer versions. It is not compatible with older AVLite
releases that expose only the flat sensor fields. Recommended plugin profiles
are supported by AVLite 0.6.4 and newer.

## Install with AVLite

Install `avlite-bridge-carla` from AVLite's Plugins browser. AVLite installs the
repository into its managed plugin directory and registers the installed path;
no machine-specific path belongs in this repository or its recommended profile.

This repository ships [avlite-bridge-carla.yaml](avlite-bridge-carla.yaml). On
plugin Install or Update, AVLite offers to import it as the
`avlite-bridge-carla` profile. Accept that prompt, select the profile, and start
the stack after launching a compatible CARLA server.

The recommended profile mirrors the tested Town10HD planning setup:

- `Carla4Bridge` with `SyncExecuter` at a 0.05-second fixed step;
- RGB camera and 3D LiDAR enabled;
- CARLA ground-truth detection, tracking, and localization enabled;
- `PerceptionPipeline` retained for prediction;
- AVLite's bundled `Town10HD_Opt.xodr` map and recorded Town10HD route;
- `HDMapGlobalPlanner`, `ShortestPathLatticePlanner`, and `StanleyController`;
- no external perception or planning plugin required;
- plugin settings stored under the registry name `avlite-bridge-carla`.

AVLite treats the self-referencing plugin locator as a portable sentinel and
resolves it to the managed install directory on each machine:

```yaml
c69_apps:
  c62_community_plugins:
    avlite-bridge-carla: avlite-bridge-carla
```

The CARLA Python API must match the running CARLA server. Install that API from
the CARLA release rather than from this repository.

## Synchronous execution

The recommended profile configures:

```yaml
c40_execution:
  c40_bridge: Carla4Bridge
  c40_controller: StanleyController
  c40_executer_type: SyncExecuter
  c40_global_planner: HDMapGlobalPlanner
  c40_global_trajectory: data/20260702_045340_global_plan.json
  c40_local_planner: ShortestPathLatticePlanner
  c40_map: data/Town10HD_Opt.xodr
  c40_pace_sim: true
  c40_sim_dt: 0.05
  c41_world_capabilities: [CAMERA_RGB, LIDAR_3D]
  c41_world_stack_capabilities: [DETECTION, TRACKING, LOCALIZATION]
plugins:
  avlite-bridge-carla:
    sync_mode: true
    sensor_timeout: 2.0
    sensor_queue_size: 64
    max_sensor_age: 0.5
    seed: 0
```

In synchronous mode, the bridge takes CARLA's fixed step from `c40_sim_dt`.
Simulation pacing must be enabled, and each explicit `dt` must match that fixed
step. Recreate the bridge after changing `c40_sim_dt`; reset it after a failed
sensor wait before advancing again.

See [SYNC_ASYNC.md](SYNC_ASYNC.md) for the synchronization, timeout, clock, and
lifecycle contracts.

## Read a synchronized capture

```python
frame = bridge.get_sensor_frame()
rgb = frame.get_camera("front").rgb
points = frame.get_lidar("roof").points
camera_time = frame.camera.stamp
lidar_time = frame.lidar.stamp
camera_by_device_id = frame.get_camera(frame.camera.sensor_id)
```

Each nonempty capture contains camera and LiDAR readings from the same CARLA
frame and timestamp. Initial captures retain sensor mounts/calibration but have
`None` readings. `frame.carla_frame` is the CARLA frame number, while
`frame.frame_id` remains the coordinate-frame label (`"base_link"`).

`CarlaSensorFrame` extends `SensorFrame` with two simulator-specific fields:
`carla_frame` and `base_to_map`, the capture-time ground-truth body pose. For
map-point projection into that image:

```python
world_to_optical = bridge.world_to_camera(frame)
```

For LiDAR-to-camera projection, use the static mounts; the current ego pose is
not required:

```python
lidar_to_optical = (
    np.linalg.inv(frame.camera.base_to_sensor) @ frame.lidar.base_to_sensor
)
```

Do not replace capture-time geometry with `bridge.get_ego_state()` at processing
time. `base_to_map` includes CARLA roll, pitch, and elevation; AVLite's generic
estimated `EgoState.pose_matrix` remains yaw-only. Sensor payload and calibration
arrays are read-only and remain stable across later callbacks.

## Tests

With a compatible AVLite checkout installed or on `PYTHONPATH`:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
python -m pytest -q -p no:cacheprovider test
```

The tests use fake CARLA I/O; they do not require a CARLA server, GPU, GUI, or
network. A live CARLA/GPU integration run is still required before deployment.
