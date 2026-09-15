# CARLA bridge — editable Documents copy

This branch starts from the user's sync-capable Carla4Bridge and targets the
**named-sensor AVLite API** in /home/av_mz/Downloads/murad/avlite.
It does not target unmodified upstream AVLite 0.6.1's flat SensorFrame.
The Desktop/Github/avlite-bridge-carla checkout is an unchanged upstream reference.

## Register this copy

In ~/.config/avlite/carla.yaml, change ONLY the CARLA plugin locator to:

    c69_apps:
      c62_community_plugins:
        avlite-bridge-carla: /home/av_mz/Documents/avlite-bridge-carla

Keep the class selection as Carla4Bridge. Using the bare plugin name still follows
the shared installation link, which currently points to the Desktop checkout.
This edit does not require moving or deleting either checkout.

For synchronous AVLite execution, use:

    c40_execution:
      c40_bridge: Carla4Bridge
      c40_executer_type: SyncExecuter
      c40_pace_sim: true
      c40_sim_dt: 0.05
      c40_control_dt: 0.05
    plugins:
      avlite_bridge_carla:
        sync_mode: true
        sensor_timeout: 2.0
        sensor_queue_size: 64
        max_sensor_age: 0.5
        seed: 0

Merge these fields into the existing sections; do not replace the whole profile.
The plugin loader normalizes the hyphenated name to avlite_bridge_carla for the
plugins section. No separate legacy plugin YAML is needed.

In synchronous mode, CARLA's fixed step comes directly from c40_sim_dt when the
bridge is created. Remove any old plugin fixed_delta_seconds setting; the bridge
constructor no longer accepts that argument either. Simulation pacing must be
enabled and c40_sim_dt must be finite and positive. The bridge also validates dt
before applying control/ticking, catching runtime mismatches. Recreate the bridge
after changing c40_sim_dt. After a failed sensor wait, reset before advancing again.

Launch from the modified AVLite environment:

    cd /home/av_mz/Downloads/murad/avlite
    source .venv/bin/activate
    python -m avlite

Use whichever virtual environment actually contains your modified AVLite.
The CARLA Python API must match your running CARLA server. This checkout is loaded
by AVLite as a plugin directory; it is not a standalone pip-installable package.

## Read a synchronized capture

    frame = bridge.get_sensor_frame()
    rgb = frame.get_camera("front").rgb
    points = frame.get_lidar("roof").points
    camera_time = frame.camera.stamp
    lidar_time = frame.lidar.stamp
    camera_by_device_id = frame.get_camera(frame.camera.sensor_id)

Each nonempty capture contains camera/LiDAR from exactly the same CARLA frame and
timestamp. Initial frames keep mounts/calibration but have None readings.
frame.carla_frame is the CARLA frame number. frame.frame_id is "base_link".

CarlaSensorFrame is a SensorFrame subclass with just two simulator-specific
metadata fields: carla_frame and base_to_map (capture-time ground-truth body pose).
It does not introduce separate sample/readings/calibration wrapper hierarchies.

For map-point projection into that image:

    world_to_optical = bridge.world_to_camera(frame)

For LiDAR-to-camera projection, ego pose is unnecessary: use the static mounts.

    lidar_to_optical = np.linalg.inv(frame.camera.base_to_sensor) @ frame.lidar.base_to_sensor

Do not replace capture-time geometry with bridge.get_ego_state() at processing
time. base_to_map includes CARLA roll, pitch and elevation; AVLite's generic
estimated EgoState.pose_matrix remains yaw-only. Capture ground truth must not be
fed into a localization algorithm that is supposed to estimate the pose.

Sensor payload/calibration arrays are read-only and stable across later callbacks.
Returned sensor wrappers are fresh; disabling capabilities cannot alter cached
captures. Explicitly copy arrays if downstream code needs to mutate them.
frame.stamp is None: assembly time is not fabricated from sensor freshness.

See SYNC_ASYNC.md for synchronization, timeout, lifetime and clock semantics.
See UPSTREAM_REVIEW.md for findings against pinned, unmodified upstream commits.

## Regression tests

No CARLA server, GUI or network is used by these tests.

    PYTHONPATH=/home/av_mz/Downloads/murad/avlite \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
    python3 -m pytest -q -p no:cacheprovider test

Tests cover delayed/out-of-order callbacks, exact frame matching, bounded queues,
timeout latching, stable snapshots, filtering, capture geometry, generation reset,
close/reload lifecycle, spawning, brake range, and real SyncExecuter clock integration.

## Remaining external integration work

The saved carla profile also loads perception_avlite/PerceptionCoreDetection.
That adapter still uses sensors.rgb, sensors.camera_params and a raw sensors.lidar.
It must be migrated separately before that camera-fusion pipeline can run with the
named-sensor core. This work does not alter that external perception repository.

The general AVLite core still has issues documented in UPSTREAM_REVIEW.md.
Bridge synchronization does not make an asynchronously shared PerceptionModel
atomic, fix map-height filtering, or give core consumers roll/pitch support.
Use SyncExecuter for a sequential stack. A live CARLA/GPU run is still required.
