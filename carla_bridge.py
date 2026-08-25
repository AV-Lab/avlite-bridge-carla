from avlite import (
    AgentState,
    ControlCommand,
    ControlStrategy,
    DepthImage,
    EgoState,
    LidarCloud,
    PerceptionModel,
    RgbImage,
    SensorFrame,
    StackCapability,
    WorldBridge,
    WorldCapability,
)
from avlite.c50_common.c52_world_sensor_datatypes import CameraParams
from typing import Union
import math
import logging
import numpy as np
import time
import threading
from typing import Optional

from .settings import PluginSettings

log = logging.getLogger(__name__)

# LiDAR sensor defaults
LIDAR_CHANNELS = 32
LIDAR_RANGE = 100.0          # metres
LIDAR_POINTS_PER_SECOND = 500_000
LIDAR_ROTATION_FREQUENCY = 10  # Hz
LIDAR_UPPER_FOV = 10.0
LIDAR_LOWER_FOV = -30.0
LIDAR_Z_OFFSET = 2.4          # sensor height above vehicle origin

# Camera sensor defaults 
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FOV = 90.0
CAM_X = 1.5          # metres forward of the vehicle origin
CAM_Y = 0.0
CAM_Z = 1.4          # metres above the vehicle origin
CAM_PITCH = 0.0      # degrees; forward-facing, upright mount
CAM_YAW = 0.0

# In async mode, LiDAR and RGB arrive from independent CARLA callback threads with
# no shared clock. get_sensor_frame() warns when their frame numbers diverge past
# this — a torn RGB/LiDAR pair projects wrong whenever ego or an object is moving.
SENSOR_SKEW_TOLERANCE_FRAMES = 2

try:
    import carla
except ImportError:
    log.error("Carla module not found. Please ensure you have the Carla Python API installed if you need to integrate with Carla.")


# ------------------------------------------------------------------
# World -> ego -> camera-optical transform, for CameraParams.world_to_camera.
#
# AVLite's world frame is x-forward/y-left/z-up (REP-103); CameraParams.world_to_camera
# must land in OpenCV optical axes (x-right, y-down, z-forward). The camera is rigidly
# mounted on the moving ego vehicle, so this has to be recomposed from the ego's current
# pose every call, not cached like a fixed calibration matrix. This intentionally mirrors
# (not imports — a world bridge shouldn't depend on a specific perception plugin)
# perception_avlite/frames.py's independently-tested world_to_camera_transform(), so any
# consumer of CameraParams gets identical semantics regardless of which side computed it.
# ------------------------------------------------------------------
_MOUNT_TO_OPTICAL_AXES = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def _rotation_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _rotation_pitch_yaw(pitch: float, yaw: float) -> np.ndarray:
    """Extrinsic Rz @ Ry rotation (roll always 0 for this bridge's mount), radians."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y


def _invert_rigid_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def _world_to_camera_optical(ego_state) -> np.ndarray:
    """(4, 4) world -> camera-optical transform for the current ego pose and this
    bridge's static CAM_X/Y/Z/PITCH/YAW mount config."""
    ego_to_world = np.eye(4)
    ego_to_world[:3, :3] = _rotation_z(ego_state.theta)
    ego_to_world[:3, 3] = [ego_state.x, ego_state.y, ego_state.z]
    world_to_ego = _invert_rigid_transform(ego_to_world)

    mount_rotation = _rotation_pitch_yaw(math.radians(CAM_PITCH), math.radians(CAM_YAW))
    # A vehicle-frame direction expressed relative to the (still vehicle-oriented) mount
    # frame is R_mount^T @ v -- R_mount is orthogonal, so its inverse is its transpose.
    cam_rotation = _MOUNT_TO_OPTICAL_AXES @ mount_rotation.T
    cam_translation = -cam_rotation @ np.array([CAM_X, CAM_Y, CAM_Z])
    ego_to_cam_optical = np.eye(4)
    ego_to_cam_optical[:3, :3] = cam_rotation
    ego_to_cam_optical[:3, 3] = cam_translation

    return ego_to_cam_optical @ world_to_ego

class Carla4Bridge(WorldBridge):
    @property
    def world_capabilities(self) -> set[WorldCapability]:
        return {
            WorldCapability.CAMERA_RGB,
            WorldCapability.LIDAR_3D,
        }

    @property
    def stack_capabilities(self) -> set[StackCapability]:
        return {
            StackCapability.DETECTION,
            StackCapability.TRACKING,
            StackCapability.LOCALIZATION,
        }

    def __init__(
        self, ego_state: Optional[EgoState], host="localhost", port=2000, scene_name="/Game/Carla/Maps/Town10HD_Opt", timeout=10.0,
        controller: Optional[ControlStrategy] = None,
        reference_point: tuple[float, float] | None = None,
        sync_mode: Optional[bool] = None,
        fixed_delta_seconds: Optional[float] = None,
    ):
        self.supports_ground_truth_detection = True
        self.supports_ground_truth_localization = True
        self.reference_point = reference_point

        self.client = None
        self.world = None
        self.ego_state = ego_state
        self.controller = controller

        # Sync mode: the server only advances one fixed-size step per explicit
        # world.tick() call. Off by default to keep existing behavior.
        self.sync_mode = sync_mode if sync_mode is not None else PluginSettings.sync_mode
        self.fixed_delta_seconds = (
            fixed_delta_seconds if fixed_delta_seconds is not None else PluginSettings.fixed_delta_seconds
        )
        self._tick_lock = threading.Lock()

        # Carla stuff
        self.vehicle = None
        self.spectator = None
        self.vehicle_blueprint = None 
        self.camera_distance = 6.0 
        self.camera_height = 2.5
        self.follow_camera = True 
        self.spawn_points = []  
        self.scene_name = scene_name  

        # Sensor actors & thread-safe buffers
        self._lidar_sensor = None
        self._rgb_sensor = None
        self._depth_sensor = None
        self._lidar_lock = threading.Lock()
        self._rgb_lock = threading.Lock()
        self._depth_lock = threading.Lock()
        self._lidar_buffer: Optional[np.ndarray] = None   # (N,4) world-frame [x,y,z,intensity]
        self._rgb_buffer: Optional[np.ndarray] = None      # (H,W,3) uint8
        self._depth_buffer: Optional[np.ndarray] = None    # (H,W) float32 metres
        # (carla frame number, carla sim-time seconds) of whatever's currently in each
        # buffer above — lets __wait_for_sensor_frame() (sync) and get_sensor_frame()'s
        # skew check (async) tell whether a buffer is current or stale.
        self._lidar_stamp: Optional[tuple[int, float]] = None
        self._rgb_stamp: Optional[tuple[int, float]] = None
        # Async mode only: each _on_lidar callback delivers just the arc swept since
        # the last server tick, not a full rotation (the server free-runs at whatever
        # FPS it's hitting, decoupled from LIDAR_ROTATION_FREQUENCY). These accumulate
        # consecutive callbacks' points across one rotation period before publishing a
        # complete sweep to _lidar_buffer. Unused in sync mode, where rotation_frequency
        # is set so a single callback per tick already is a full sweep.
        self._lidar_accum: list[np.ndarray] = []
        self._lidar_sweep_start_ts: Optional[float] = None
        self._lidar_rotation_frequency: float = LIDAR_ROTATION_FREQUENCY
        self.use_static_objects = False  # Use static objects in perception model
        self.static_vehicle_labels = (
                carla.CityObjectLabel.Car,
                carla.CityObjectLabel.Truck,
                carla.CityObjectLabel.Bus,
                carla.CityObjectLabel.Motorcycle,
                carla.CityObjectLabel.Bicycle
                )

        self.supports_ground_truth_detection = True  # Carla provides ground truth state

        log.info(f"Connecting to Carla at {host}:{port}")
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            # get_available_maps() returns a boost::python::list; on this build that
            # constructor segfaults (PyList_New crash inside libcarla.so) even though
            # scalar/string RPCs like get_server_version() work fine. load_world()
            # itself raises a clean RuntimeError for an unknown map name, so skip the
            # list-returning validation call entirely rather than crash the process.
            self.world = self.client.load_world(scene_name)
            log.info(f"Connected to Carla at {host}:{port} and loaded scene {scene_name}")

            self.__configure_sync_mode()

            # Get the spectator to control the camera
            self.spectator = self.world.get_spectator()

            self.spawn_points = self.world.get_map().get_spawn_points()
            log.info(f"Found {len(self.spawn_points)} spawn points in the map")

            spawn_npc_vehicles(self.world, num_vehicles=10)
            # Initialize vehicle blueprint
            self.__initialize_vehicle_blueprint()

            # In sync mode the world only advances when control_ego_state() ticks it,
            # so the camera/state follow is driven from there instead of a free-running
            # background thread (which would poll at wall-clock rate, decoupled from sim
            # time, defeating the point of a fixed, deterministic step).
            if not self.sync_mode:
                self.start_bg_camera_and_state_update()

        except Exception as e:
            log.error(f"Failed to connect to Carla: {e}")
            log.error("Make sure the Carla simulator is running on the specified host and port.")


    def __configure_sync_mode(self):
        """Put the CARLA server (and its traffic manager) into synchronous, fixed-timestep
        mode, so the world only advances via explicit tick() calls from this bridge
        instead of the server's free-running real-time clock. No-op in async mode.
        """
        if not self.sync_mode or not self.world:
            return
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        self.world.apply_settings(settings)

        # NPC autopilot (spawn_npc_vehicles) is driven by the traffic manager, which
        # has its own sync flag independent of the world's.
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        log.info(f"Carla synchronous mode enabled (fixed_delta_seconds={self.fixed_delta_seconds})")

    def __tick(self):
        """Advance the world by one fixed step in sync mode; no-op in async mode
        (the server advances on its own real-time clock there)."""
        if not self.sync_mode or not self.world:
            return
        with self._tick_lock:
            frame = self.world.tick()
            self.__update_camera_position_and_state()
            self.__wait_for_sensor_frame(frame)

    def __wait_for_sensor_frame(self, frame, timeout=1.0):
        """Block until every attached sensor has a buffer stamped >= *frame*.

        world.tick() returns as soon as the server has stepped physics — it does not
        wait for sensor callbacks, which land slightly later on CARLA's own listener
        threads. Without this, code that reads get_lidar_data()/get_rgb_image() right
        after __tick() can still see the previous frame's data. Only meaningful in
        sync mode, where exactly one callback per sensor is expected per tick.
        """
        deadline = time.time() + timeout
        for sensor, stamp_attr, lock in (
            (self._lidar_sensor, "_lidar_stamp", self._lidar_lock),
            (self._rgb_sensor, "_rgb_stamp", self._rgb_lock),
        ):
            if sensor is None:
                continue
            while True:
                with lock:
                    stamp = getattr(self, stamp_attr)
                if stamp is not None and stamp[0] >= frame:
                    break
                if time.time() >= deadline:
                    log.warning(f"Timed out waiting for {stamp_attr} to reach frame {frame}")
                    break
                time.sleep(0.001)

    def step(self, dt: Optional[float] = 0.01) -> None:
        """Public WorldBridge hook to advance the world without a control command.

        AVLite's executers never call this today — control_ego_state() already ticks
        the world every control cycle in sync mode, so nothing extra is needed there.
        This exists for callers driving Carla4Bridge directly (e.g. a script that only
        spawns agents / reads ground truth and never calls control_ego_state()) and
        still wants sync mode's deterministic, one-step-at-a-time stepping.
        """
        self.__tick()

    def start_bg_camera_and_state_update(self, interval=0.01):
        """Start a periodic update of the camera position"""
        import threading
        import time

        def update_thread():
            while True:
                self.__update_camera_position_and_state()
                time.sleep(interval)

        camera_thread = threading.Thread(target=update_thread)
        camera_thread.daemon = True
        camera_thread.start()

    def __update_camera_position_and_state(self):
        """Update the camera position to follow behind the vehicle"""
        if not self.vehicle or not self.spectator or not self.follow_camera:
            return

        # Get the vehicle's transform
        vehicle_transform = self.vehicle.get_transform()

        # update state
        # self.get_ego_state()

        yaw_rad = vehicle_transform.rotation.yaw * (3.14159 / 180.0)
        dx = -self.camera_distance * math.cos(yaw_rad)
        dy = -self.camera_distance * math.sin(yaw_rad)

        camera_location = carla.Location(
            x=vehicle_transform.location.x + dx,
            y=vehicle_transform.location.y + dy,
            z=vehicle_transform.location.z + self.camera_height,
        )

        # Point the camera at the vehicle
        camera_rotation = carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw, roll=0)  # Look down slightly

        camera_transform = carla.Transform(camera_location, camera_rotation)
        self.spectator.set_transform(camera_transform)


    def __initialize_vehicle_blueprint(self):
        """Initialize the vehicle blueprint to be used for spawning"""
        if not self.world:
            log.error("Cannot initialize vehicle blueprint: world not connected")
            return

        blueprint_library = self.world.get_blueprint_library()

        # Print available vehicle blueprints
        vehicle_blueprints = [bp.id for bp in blueprint_library.filter("vehicle.*")]
        log.info(f"Available vehicle blueprints: {vehicle_blueprints}")

        if vehicle_blueprints:
            self.vehicle_blueprint = blueprint_library.find(vehicle_blueprints[0])
            log.info(f"Using first available vehicle: {vehicle_blueprints[0]}")
        else:
            log.error("No vehicle blueprints available in Carla")
            self.vehicle_blueprint = None

    def __spawn_vehicle(self, state: Union[EgoState, AgentState]):
        """Spawn the ego vehicle at the given state position"""
        if not self.world or not self.vehicle_blueprint:
            log.error("Cannot spawn vehicle: world not connected or blueprint not initialized")
            return

        # Use a valid spawn point from Carla
        if self.spawn_points:
            # Find the closest spawn point to the requested state
            closest_point = None
            min_distance = float("inf")
            for point in self.spawn_points:
                distance = ((point.location.x - state.x) ** 2 + (point.location.y - state.y) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point

            # If we're too far from any spawn point, just use the first one
            if min_distance > 100.0:  # If more than 100 meters away
                log.warning(f"Requested position is too far from any valid spawn point. Using first spawn point.")
                spawn_point = self.spawn_points[0]
            else:
                spawn_point = closest_point

            log.info(
                f"Using spawn point at ({spawn_point.location.x}, {spawn_point.location.y}, {spawn_point.location.z})"
            )

        else:
            log.warning("No spawn points found in Carla map! Using arbitrary spawn point.")
            spawn_point = carla.Transform(carla.Location(x=state.x, y=state.y, z=1.0))

        # Try to spawn the vehicle. spawn_actor() raises RuntimeError on collision
        # instead of returning None, which would skip the fallback loop below entirely
        # (the exception propagates straight out of __spawn_vehicle) — try_spawn_actor()
        # fails gracefully instead, so a collision here can actually reach the retry loop.
        self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, spawn_point)

        # If spawning fails, try other spawn points
        if not self.vehicle and self.spawn_points:
            log.warning("Failed to spawn at selected point. Trying other spawn points.")
            for i, spawn_point in enumerate(self.spawn_points):
                self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, spawn_point)
                if self.vehicle:
                    log.info(f"Successfully spawned at alternative point {i}")
                    # Update the state to match the spawn point
                    state.x = spawn_point.location.x
                    state.y = spawn_point.location.y
                    state.theta = spawn_point.rotation.yaw * (3.14159 / 180.0)
                    break

            if not self.vehicle:
                log.error("Failed to spawn vehicle at any spawn point!")

        # Attach sensors once vehicle exists
        if self.vehicle:
            self.__attach_sensors()

    # ------------------------------------------------------------------
    # Sensor lifecycle
    # ------------------------------------------------------------------
    def __attach_sensors(self):
        """Spawn LiDAR (and optionally camera) sensors attached to the ego vehicle."""
        if not self.vehicle or not self.world:
            return
        bp_lib = self.world.get_blueprint_library()

        # --- LiDAR ---
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(LIDAR_CHANNELS))
        lidar_bp.set_attribute('range', str(LIDAR_RANGE))
        lidar_bp.set_attribute('points_per_second', str(LIDAR_POINTS_PER_SECOND))
        if self.sync_mode:
            # A full 360-degree sweep must complete in exactly one tick, or
            # get_lidar_data() only ever returns partial (half, third, ...) sweeps.
            lidar_rotation_frequency = 1.0 / self.fixed_delta_seconds
        else:
            lidar_rotation_frequency = LIDAR_ROTATION_FREQUENCY
        self._lidar_rotation_frequency = lidar_rotation_frequency
        lidar_bp.set_attribute('rotation_frequency', str(lidar_rotation_frequency))
        lidar_bp.set_attribute('upper_fov', str(LIDAR_UPPER_FOV))
        lidar_bp.set_attribute('lower_fov', str(LIDAR_LOWER_FOV))
        lidar_transform = carla.Transform(carla.Location(z=LIDAR_Z_OFFSET))
        self._lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        self._lidar_sensor.listen(self._on_lidar)
        log.info(f"LiDAR sensor attached ({LIDAR_CHANNELS}ch, {LIDAR_RANGE}m range, "
                 f"{lidar_rotation_frequency}Hz rotation)")

        rgb_bp = bp_lib.find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(CAMERA_WIDTH))
        rgb_bp.set_attribute('image_size_y', str(CAMERA_HEIGHT))
        rgb_bp.set_attribute('fov', str(CAMERA_FOV))
        # Capture every simulation step rather than gating on wall-clock time, so the
        # camera stays aligned with LiDAR (and with tick boundaries in sync mode).
        rgb_bp.set_attribute('sensor_tick', '0.0')
        camera_transform = carla.Transform(carla.Location(x=CAM_X, y=CAM_Y, z=CAM_Z),
                                            carla.Rotation(pitch=CAM_PITCH, yaw=CAM_YAW))
        self._rgb_sensor = self.world.spawn_actor(rgb_bp, camera_transform, attach_to=self.vehicle)
        self._rgb_sensor.listen(self._on_rgb)

    def __destroy_sensors(self):
        """Destroy all sensor actors."""
        for sensor in (self._lidar_sensor, self._rgb_sensor, self._depth_sensor):
            if sensor is not None:
                try:
                    sensor.stop()
                    sensor.destroy()
                except Exception as e:
                    log.warning(f"Error destroying sensor: {e}")
        self._lidar_sensor = None
        self._rgb_sensor = None
        self._depth_sensor = None
        with self._lidar_lock:
            self._lidar_buffer = None
            self._lidar_stamp = None
        self._lidar_accum = []
        self._lidar_sweep_start_ts = None
        with self._rgb_lock:
            self._rgb_buffer = None
            self._rgb_stamp = None
        with self._depth_lock:
            self._depth_buffer = None

    # ------------------------------------------------------------------
    # Sensor callbacks (run on Carla's sensor thread)
    # ------------------------------------------------------------------
    def _on_lidar(self, measurement):
        """Convert carla.LidarMeasurement to world-frame (N,4) numpy array with AVLite coord convention."""
        # Raw data: each point is [x, y, z, intensity] in sensor-local frame
        data = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()

        # Sensor → world transform
        st = measurement.transform
        yaw = math.radians(st.rotation.yaw)
        pitch = math.radians(st.rotation.pitch)
        roll = math.radians(st.rotation.roll)

        # Rotation matrix (Carla uses left-hand UE4 convention)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [  -sp,           cp*sr,           cp*cr  ],
        ], dtype=np.float32)

        pts_local = data[:, :3]                       # (N,3)
        pts_world = pts_local @ R.T                   # rotate to world
        pts_world[:, 0] += st.location.x
        pts_world[:, 1] += st.location.y
        pts_world[:, 2] += st.location.z

        # Carla LH → AVLite RH: negate Y
        pts_world[:, 1] *= -1.0

        result = np.column_stack([pts_world, data[:, 3]])  # (N,4)

        if self.sync_mode:
            # rotation_frequency is set to 1/fixed_delta_seconds, so this single
            # callback already is a complete 360° sweep — no accumulation needed.
            with self._lidar_lock:
                self._lidar_buffer = result
                self._lidar_stamp = (measurement.frame, measurement.timestamp)
            return

        # Async mode: this callback only covers the arc swept since the last server
        # tick, whatever that tick's wall-clock duration happened to be. Accumulate
        # consecutive callbacks until a full rotation period has elapsed, then publish
        # the concatenated sweep and start accumulating the next one.
        if self._lidar_sweep_start_ts is None:
            self._lidar_sweep_start_ts = measurement.timestamp
        self._lidar_accum.append(result)

        rotation_period = 1.0 / self._lidar_rotation_frequency
        if measurement.timestamp - self._lidar_sweep_start_ts >= rotation_period:
            complete_sweep = np.concatenate(self._lidar_accum, axis=0)
            with self._lidar_lock:
                self._lidar_buffer = complete_sweep
                self._lidar_stamp = (measurement.frame, measurement.timestamp)
            self._lidar_accum = []
            self._lidar_sweep_start_ts = measurement.timestamp

    def _on_rgb(self, image):
        """Convert carla.Image (BGRA) to SensorFrame's (H,W,3) uint8 RGB convention."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        rgb = arr[:, :, 2::-1]  # BGRA -> RGB, drop alpha
        with self._rgb_lock:
            self._rgb_buffer = rgb.copy()
            self._rgb_stamp = (image.frame, image.timestamp)

    # ------------------------------------------------------------------
    # WorldBridge sensor overrides
    # ------------------------------------------------------------------
    def get_lidar_data(self) -> Optional[LidarCloud]:
        """Return latest LiDAR point cloud as (N,4) [x,y,z,intensity] in AVLite world frame."""
        with self._lidar_lock:
            return self._lidar_buffer

    def get_rgb_image(self) -> Optional[RgbImage]:
        with self._rgb_lock:
            return self._rgb_buffer

    def get_depth_image(self) -> Optional[DepthImage]:
        with self._depth_lock:
            return self._depth_buffer

    def get_camera_intrinsics(self) -> Optional[np.ndarray]:
        """3x3 intrinsic matrix K, derived from the same FOV/resolution
        __attach_sensors() uses to actually spawn the camera."""
        focal_length = CAMERA_WIDTH / (2.0 * math.tan(math.radians(CAMERA_FOV) / 2.0))
        cx, cy = CAMERA_WIDTH / 2.0, CAMERA_HEIGHT / 2.0
        return np.array([
            [focal_length, 0.0,          cx],
            [0.0,          focal_length, cy],
            [0.0,          0.0,          1.0],
        ], dtype=np.float32)
    
    def get_camera_extrinsics(self) -> Optional[np.ndarray]:
        """4x4 camera-mount-relative-to-ego transform, built from the same
        CAM_X/CAM_Y/CAM_Z/CAM_PITCH/CAM_YAW constants __attach_sensors() mounts
        the camera with. This is the static mount offset, not a per-tick world
        pose — unlike LiDAR/RGB it never needs recomputing after a tick.
        """
        yaw, pitch = math.radians(CAM_YAW), math.radians(CAM_PITCH)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        # Carla LH rotation (roll=0, matching the mount in __attach_sensors)
        R_lh = np.array([
            [cy * cp, -sy, cy * sp],
            [sy * cp,  cy, sy * sp],
            [    -sp, 0.0,      cp],
        ], dtype=np.float32)
        t_lh = np.array([CAM_X, CAM_Y, CAM_Z], dtype=np.float32)
        # Carla LH -> AVLite RH: conjugate by the Y-reflection F=diag(1,-1,1) so
        # the result stays a proper (det=+1) rotation in the new frame. Negating
        # just the translation's Y (as done for lidar points/ego state elsewhere
        # in this file) is only correct for points, not for a full rotation
        # matrix — moot today since CAM_PITCH=CAM_YAW=0 makes R_lh the identity,
        # but wrong in general if either mount angle is ever changed.
        F = np.diag([1.0, -1.0, 1.0]).astype(np.float32)
        R_rh = F @ R_lh @ F
        t_rh = F @ t_lh
        extrinsic = np.eye(4, dtype=np.float32)
        extrinsic[:3, :3] = R_rh
        extrinsic[:3, 3] = t_rh
        return extrinsic

    def get_camera_params(self) -> Optional[CameraParams]:
        """Full CameraParams (intrinsic + world->camera-optical), recomputed from
        the ego's current pose every call — see _world_to_camera_optical()."""
        if self.ego_state is None:
            return None
        return CameraParams(
            intrinsic=self.get_camera_intrinsics(),
            world_to_camera=_world_to_camera_optical(self.ego_state),
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
        )

    def get_sensor_frame(self) -> SensorFrame:
        """Return an atomic snapshot of buffered sensor data.

        In sync mode __tick() already waits for both buffers to reach the current
        tick's frame, so they're expected to match here. In async mode there's no
        such guarantee — RGB and LiDAR arrive independently off the wall clock — so
        this only warns on skew rather than blocking or dropping data.
        """
        with self._rgb_lock:
            rgb = self._rgb_buffer
            rgb_stamp = self._rgb_stamp
        with self._depth_lock:
            depth = self._depth_buffer
        with self._lidar_lock:
            lidar = self._lidar_buffer
            lidar_stamp = self._lidar_stamp

        if rgb_stamp is not None and lidar_stamp is not None:
            frame_skew = abs(rgb_stamp[0] - lidar_stamp[0])
            if frame_skew > SENSOR_SKEW_TOLERANCE_FRAMES:
                log.warning(
                    f"RGB/LiDAR frame skew ({frame_skew} frames, rgb={rgb_stamp[0]} "
                    f"lidar={lidar_stamp[0]}) exceeds tolerance "
                    f"({SENSOR_SKEW_TOLERANCE_FRAMES}); this SensorFrame's rgb/lidar "
                    f"pair may not describe the same instant."
                )

        # Sim-time acquisition stamp for the frame as a whole — the newer of the two
        # if both are present, so a consumer checking staleness sees the worst case.
        if rgb_stamp is not None and lidar_stamp is not None:
            stamp = max(rgb_stamp[1], lidar_stamp[1])
        elif rgb_stamp is not None:
            stamp = rgb_stamp[1]
        elif lidar_stamp is not None:
            stamp = lidar_stamp[1]
        else:
            stamp = None

        return SensorFrame(
            rgb=rgb, depth=depth, lidar=lidar, stamp=stamp,
            camera_params=self.get_camera_params(),
        )

    def control_ego_state(self, cmd: ControlCommand, dt=0.01):
        """Update the ego state with the given command.
        This method applies control commands to the vehicle and updates the state.
        If the vehicle doesn't exist yet, it will be spawned.
        """
        # If vehicle doesn't exist, spawn it
        if not self.vehicle:
            self.__spawn_vehicle(self.ego_state)

        log.debug(f"Applying control: {cmd}")
        assert self.ego_state is not None, "Ego state is None. Cannot update state without a reference."

        current_velocity = self.ego_state.velocity

        # Calculate throttle and brake values
        throttle = np.abs(cmd.acceleration) / (self.controller.ego_max_acceleration if self.controller is not None else 10.0) if cmd.acceleration > 0 else 0.0
        brake = np.abs(cmd.acceleration) / (self.controller.ego_min_acceleration if self.controller is not None else -20.0) if cmd.acceleration < 0 else 0.0

        # Convert to float to ensure correct type
        throttle = float(throttle)
        brake = float(brake)
        steer = float(-cmd.steer)

        # Determine reverse state
        is_nearly_stopped = current_velocity < 0.1  # threshold for "stopped"
        wants_reverse = cmd.acceleration < 0
        is_reverse = wants_reverse and is_nearly_stopped

        # In reverse mode, use throttle instead of brake for backward movement
        if is_reverse and wants_reverse:
            throttle = float(np.abs(cmd.acceleration) / (self.controller.ego_max_acceleration if self.controller is not None else 10.0))
            brake = 0.0

        # When steering with zero throttle, maintain a small throttle to prevent stopping
        if throttle == 0.0 and brake == 0.0 and abs(cmd.steer) > 0.01:
            throttle = 0.05  # Small throttle value to maintain momentum during steering

        log.debug(f"Velocity: {current_velocity}, Throttle: {throttle}, Brake: {brake}, Reverse: {is_reverse}")

        # Ensure all parameters are of the correct type for the Carla API
        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, reverse=bool(is_reverse))
        self.vehicle.apply_control(control)

        # In sync mode the server won't apply that control (or advance physics) until
        # we tick it; do so now so get_ego_state() below reflects this frame's update.
        self.__tick()

        # Update self.ego_state from vehicle
        self.get_ego_state()
    

    def teleport_ego(self, x: float, y: float, theta: Optional[float] = None):
        if not self.vehicle:
            self.__spawn_vehicle(self.ego_state)
        self.ego_state.x = x
        self.ego_state.y = y
        if theta is not None:
            self.ego_state.theta = -theta # theta is inversed in Carla and UE

        if self.vehicle:
            # Convert theta from radians to degrees for Carla
            theta_deg = self.ego_state.theta * (180.0 / 3.14159) if theta else None
            transform = carla.Transform(
                carla.Location(x=x, y=-y, z=1.0),
                carla.Rotation(yaw=theta_deg) if theta_deg is not None else carla.Rotation()
            )
            self.vehicle.set_transform(transform)

    
    # TODO: Carla transformation
    def get_ego_state(self):
        """Get the current state of the ego vehicle.
        The method handles the difference of left-hand rule of Carla to right-hand rule of AVLite. 
        """
        if not self.vehicle:
            self.__spawn_vehicle(self.ego_state)
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        
        # Log the raw transform data for debugging
        # log.debug(f"Vehicle Transform: Location({transform.location.x}, {transform.location.y}, {transform.location.z}), "
                  # f"Rotation({transform.rotation.pitch}, {transform.rotation.yaw}, {transform.rotation.roll})")
        
        self.ego_state.x = transform.location.x
        self.ego_state.y = -1*transform.location.y
        self.ego_state.theta = -transform.rotation.yaw * (3.14159 / 180.0)
        self.ego_state.velocity = (velocity.x**2 + velocity.y**2) ** 0.5
        log.debug(f"Updated Ego State: x={self.ego_state.x}, y={self.ego_state.y}, theta={self.ego_state.theta}, velocity={self.ego_state.velocity}")
#
        # self.__update_camera_position_and_state()


        return self.ego_state

    def spawn_agent(self, agent_state: AgentState):
        """Spawn an agent in the Carla simulator.
        This method handles the spawning of agents in the Carla simulator.
        It uses the agent's state to determine the spawn point and vehicle type.
        """
        self.__spawn_vehicle(agent_state)

    def get_ground_truth_perception_model(self) -> PerceptionModel:
        agents: list[AgentState] = []
        # log.info("Collecting ground truth perception model from Carla...")

        agents = []
        
        if self.world:
            # log.info(f"world actors are {self.world.get_actors().filter('vehicle.*')}")

            for actor in self.world.get_actors().filter('vehicle.*'):
                if self.vehicle and actor.id == self.vehicle.id:
                    continue
                    
                # Get vehicle data
                transform = actor.get_transform()
                velocity = actor.get_velocity()
                bbox = actor.bounding_box
                
                # Create agent state with coordinate conversion (Carla to AVLite)
                agent = AgentState(
                    x=transform.location.x,
                    y=-transform.location.y,  # Y-axis inversion
                    theta=-math.radians(transform.rotation.yaw),
                    velocity=math.sqrt(velocity.x**2 + velocity.y**2),
                    agent_id=int(actor.id),
                    length=bbox.extent.x * 2,
                    width=bbox.extent.y * 2
                )
                agents.append(agent)

                if self.use_static_objects:
                    static_agents = self.get_static_objects()
                    agents.extend(static_agents)
        return PerceptionModel(ego_vehicle=self.ego_state, agent_vehicles=agents)

    def get_static_objects(self):
        agents: list[AgentState] = []
        static_obj_bboxes = []
        for label in self.static_vehicle_labels:
                static_obj_bboxes.extend(self.world.get_level_bbs(label))

        for bbox in static_obj_bboxes:
                # Convert static bounding box to AgentState
                static_agent = AgentState(
                    x=bbox.location.x,
                    y=-bbox.location.y,  # Y-axis inversion
                    theta=-math.radians(bbox.rotation.yaw),
                    velocity=0.0,  # Static objects have no velocity
                    agent_id=-1,  # Use -1 for static objects
                    length=bbox.extent.x * 2,
                    width=bbox.extent.y * 2
                )
                agents.append(static_agent)

        return agents
    
    def reset(self):
        """Reset the simulator and state.
        This method destroys the current vehicle, resets the simulation,
        and prepares the environment for a new run.
        """
        log.info("Resetting Carla simulation...")

        # Destroy sensors before the vehicle
        self.__destroy_sensors()

        # Destroy the current vehicle if it exists
        if self.vehicle:
            try:
                self.vehicle.destroy()
                log.info("Destroyed existing vehicle")
            except Exception as e:
                log.error(f"Error destroying vehicle: {e}")
            finally:
                self.vehicle = None

        # Destroy all other actors that might have been created
        # (like other vehicles, sensors, etc.)
        if self.world:
            try:
                for actor in self.world.get_actors():
                    # Only destroy actors that are vehicles (not the spectator, etc.)
                    if "vehicle" in actor.type_id:
                        actor.destroy()
                log.info("Destroyed all vehicle actors")
            except Exception as e:
                log.error(f"Error destroying actors: {e}")

        # Reset camera transforms
        self.current_camera_transform = None
        self.target_camera_transform = None

        # Reset the world's state if possible
        if self.client:
            try:
                # world.tick() is only valid in sync mode; in async mode the server
                # advances on its own and this call would raise.
                if self.sync_mode:
                    self.world.tick()

                # Reset the simulation to its initial state. reset_settings=False keeps
                # our synchronous_mode/fixed_delta_seconds instead of reload_world()
                # silently reverting the world to its (async) defaults.
                self.world = self.client.reload_world(reset_settings=False)

                # Get the spectator again after world reload
                self.spectator = self.world.get_spectator()

                # Refresh spawn points
                self.spawn_points = self.world.get_map().get_spawn_points()

                # Set weather to clear day again
                weather = carla.WeatherParameters.ClearNoon
                self.world.set_weather(weather)

                # reset_settings=False preserves carla.WorldSettings (synchronous_mode/
                # fixed_delta_seconds), but the traffic manager is a separate CARLA
                # subsystem with its own sync flag that reload_world() makes no
                # documented guarantee about — re-apply rather than assume it survived.
                self.__configure_sync_mode()

                # __init__ always seeds the world with NPCs; reload_world() destroys
                # them and reset() has no other way to bring them back, so respawn here
                # too rather than leaving the world permanently empty after a reset.
                spawn_npc_vehicles(self.world, num_vehicles=10)

                # Re-initialize the vehicle blueprint
                self.__initialize_vehicle_blueprint()

                log.info("Carla world reset complete")
            except Exception as e:
                log.error(f"Error resetting world: {e}")

        log.info("Reset complete")
    


def spawn_npc_vehicles(world, num_vehicles=10):
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    spawn_points = world.get_map().get_spawn_points()
    import random

    for i in range(min(num_vehicles, len(spawn_points))):
        bp = random.choice(vehicle_blueprints)
        transform = spawn_points[i]
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle:
            vehicle.set_autopilot(True)




def draw_actor_bbox(world,static_car_bboxes, color=None, life_time=0.05, thickness=0.05):
    """
    Draw bounding boxes around all vehicles in the CARLA world.
    
    Args:
        world: CARLA world instance
        static_car_bboxes: List of static vehicle bounding boxes
        color: Color for dynamic vehicles (default: red)
        life_time: Duration boxes remain visible in seconds (default: 0.05)
        thickness: Line thickness of bounding box edges (default: 0.05)
    
    Note: Dynamic vehicles are drawn in the specified color (default red), static vehicles in blue.
    """
    if color is None: # default value
        color = carla.Color(255, 0, 0)

    # spawned actors
    for actor in world.get_actors().filter('vehicle.*'):
        # Get the bounding box and transform
        bbox = actor.bounding_box
        transform = actor.get_transform()
        
        # Transform bounding box to world coordinates
        bbox.location = transform.transform(bbox.location)
        bbox.rotation = transform.rotation
        
        # Draw the bounding box
        world.debug.draw_box(
            box=bbox,
            rotation=transform.rotation,
            thickness=thickness,
            color=color,
            life_time=life_time
        )
    
    # static actors 
    for bbox in static_car_bboxes:
            world.debug.draw_box(
                box=bbox,
                rotation=bbox.rotation,
                thickness=thickness,
                color=carla.Color(0, 0, 255),
                life_time=life_time
            )
    

