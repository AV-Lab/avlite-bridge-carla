"""CARLA bridge for AVLite's named Camera/Lidar SensorFrame API.

One owner ticks CARLA. Callbacks publish exact-frame camera/LiDAR pairs;
getters never advance the simulator. See SYNC_ASYNC.md for clock contracts.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import atexit
import logging
import math
import random
import threading
import time

import numpy as np

from avlite import (
    AgentState, Camera, ControlCommand, ControlStrategy, EgoState, Lidar,
    PerceptionModel, SensorFrame, StackCapability, WorldBridge, WorldCapability,
)
from avlite.c10_perception.c11_perception_model import EGO_AGENT_ID
from .settings import PluginSettings

try:
    import carla
except ImportError:
    carla = None

log = logging.getLogger(__name__)
# importlib.reload retains module globals. Preserve ownership across GUI reloads.
_ACTIVE_BRIDGES = globals().get("_ACTIVE_BRIDGES", {})


def _close_active_bridges():
    for bridge in list(_ACTIVE_BRIDGES.values()):
        try:
            bridge.close()
        except Exception:
            log.exception("Could not close CARLA bridge during shutdown")


if not globals().get("_EXIT_HOOK_REGISTERED", False):
    atexit.register(_close_active_bridges)
    _EXIT_HOOK_REGISTERED = True

LIDAR_CHANNELS = 32
LIDAR_RANGE = 100.0
LIDAR_POINTS_PER_SECOND = 500_000
LIDAR_ROTATION_FREQUENCY = 10
LIDAR_UPPER_FOV = 10.0
LIDAR_LOWER_FOV = -30.0
LIDAR_Z_OFFSET = 2.4
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV = 1280, 720, 90.0
CAM_X, CAM_Y, CAM_Z = 1.5, 0.0, 1.4
CAM_PITCH, CAM_YAW = 0.0, 0.0
_REFLECTION = np.diag([1.0, -1.0, 1.0, 1.0])
# Optical (right, down, forward) -> CARLA mount (forward, right, up).
_OPTICAL_TO_CARLA = np.array([
    [0., 0., 1., 0.], [1., 0., 0., 0.],
    [0., -1., 0., 0.], [0., 0., 0., 1.],
])


def _readonly(array):
    array = np.array(array, copy=True)
    array.setflags(write=False)
    return array


@dataclass
class CarlaSensorFrame(SensorFrame):
    """An ordinary SensorFrame plus CARLA capture metadata.

    base_to_map is the ego body pose at acquisition, including roll/pitch.
    It is ground truth for projection/debugging, NOT a localization estimate.
    SensorFrame.frame_id remains the coordinate-frame label, not a tick number.
    """
    carla_frame: int | None = None
    base_to_map: np.ndarray | None = None


class Carla4Bridge(WorldBridge):
    @property
    def world_capabilities(self):
        return {WorldCapability.CAMERA_RGB, WorldCapability.LIDAR_3D}

    @property
    def stack_capabilities(self):
        return {StackCapability.DETECTION, StackCapability.TRACKING, StackCapability.LOCALIZATION}

    def __init__(
        self, ego_state: EgoState | None, host="localhost", port=2000,
        scene_name="/Game/Carla/Maps/Town10HD_Opt", timeout=10.0,
        controller: ControlStrategy | None = None, reference_point=None,
        sync_mode: bool | None = None,
    ):
        if carla is None:
            raise ImportError("Install the CARLA Python API matching your CARLA server.")
        self.ego_state = ego_state if ego_state is not None else EgoState()
        self.controller, self.reference_point = controller, reference_point
        self.supports_ground_truth_detection = self.supports_ground_truth_localization = True
        self.sync_mode = PluginSettings.sync_mode if sync_mode is None else sync_mode
        self.fixed_delta_seconds = None
        if self.sync_mode:
            from avlite.c40_execution.c49_settings import ExecutionSettings
            # AVLite owns the simulation clock; CARLA uses the same step duration.
            self.fixed_delta_seconds = ExecutionSettings.c40_sim_dt
        self.validate_avlite_timing()
        self.sensor_timeout = PluginSettings.sensor_timeout
        self.sensor_queue_size = PluginSettings.sensor_queue_size
        self.max_sensor_age = PluginSettings.max_sensor_age
        self.seed = PluginSettings.seed
        self._tick_lock = threading.RLock()
        self._sensor_condition = threading.Condition()
        self._sensor_generation = 0
        self._pending = {}
        self._completed = {}
        self._latest_frame = None
        self._last_delivery = None
        self._sensor_started_at = None
        self._sync_fault = None
        self._closed = False
        self._stop_event = threading.Event()
        self._camera_thread = None
        self.vehicle = self.world = self.client = self.spectator = None
        self._traffic_manager = self._original_settings = None
        self._npc_actors = []
        self._lidar_sensor = self._rgb_sensor = self._depth_sensor = None
        self.vehicle_blueprint = None
        self.spawn_points = []
        self.camera_distance, self.camera_height, self.follow_camera = 6.0, 2.5, True
        self.scene_name = scene_name
        self.use_static_objects = False
        self.static_vehicle_labels = tuple(
            getattr(carla.CityObjectLabel, name)
            for name in ("Car", "Truck", "Bus", "Motorcycle", "Bicycle")
        )
        self._make_sensor_templates()
        self._owner_key = (host, port)
        previous = _ACTIVE_BRIDGES.get(self._owner_key)
        if previous is not None:
            previous.close()
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            # Avoid the list-returning RPC that crashed the user's libcarla build.
            self.world = self.client.load_world(scene_name)
            self._original_settings = self.world.get_settings()
            self.__configure_sync_mode()
            if self.sync_mode:
                # Configure before the repeatable initial episode starts.
                self.world = self.client.reload_world(reset_settings=False)
                self.__configure_sync_mode()
            self._prepare_world()
            if not self.sync_mode:
                self.start_bg_camera_and_state_update()
            _ACTIVE_BRIDGES[self._owner_key] = self
        except Exception:
            self.close()
            raise

    def validate_avlite_timing(self):
        """Fail early for unsupported fixed-step AVLite configuration."""
        if not self.sync_mode:
            return
        from avlite.c40_execution.c49_settings import ExecutionSettings
        if not ExecutionSettings.c40_pace_sim:
            raise ValueError("Synchronous CARLA requires c40_pace_sim: true")
        if not math.isfinite(self.fixed_delta_seconds) or self.fixed_delta_seconds <= 0:
            raise ValueError("AVLite c40_sim_dt must be finite and positive")

    def _make_sensor_templates(self):
        mount = np.eye(4)
        mount[2, 3] = LIDAR_Z_OFFSET
        self._lidar_mount = Lidar(sensor_name="roof", base_to_sensor=_readonly(mount))
        # Use CARLA's actual transform implementation, including its pitch sign.
        camera_transform = carla.Transform(
            carla.Location(x=CAM_X, y=CAM_Y, z=CAM_Z),
            carla.Rotation(pitch=CAM_PITCH, yaw=CAM_YAW),
        )
        optical_mount = _REFLECTION @ np.asarray(camera_transform.get_matrix()) @ _OPTICAL_TO_CARLA
        focal = CAMERA_WIDTH / (2 * math.tan(math.radians(CAMERA_FOV) / 2))
        intrinsic = np.array([[focal, 0, CAMERA_WIDTH/2], [0, focal, CAMERA_HEIGHT/2], [0, 0, 1.]])
        self._camera_mount = Camera(
            sensor_name="front", intrinsic=_readonly(intrinsic),
            width=CAMERA_WIDTH, height=CAMERA_HEIGHT,
            base_to_sensor=_readonly(optical_mount),
        )

    def __configure_sync_mode(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = self.sync_mode
        settings.fixed_delta_seconds = self.fixed_delta_seconds if self.sync_mode else None
        if self.sync_mode and settings.substepping:
            if self.fixed_delta_seconds > settings.max_substep_delta_time * settings.max_substeps:
                raise ValueError("CARLA fixed step exceeds the configured physics substep budget")
        self.world.apply_settings(settings)
        self._traffic_manager = self.client.get_trafficmanager()
        self._traffic_manager.set_synchronous_mode(self.sync_mode)
        self._traffic_manager.set_random_device_seed(self.seed)

    def _prepare_world(self):
        self.spectator = self.world.get_spectator()
        self.spawn_points = self.world.get_map().get_spawn_points()
        blueprints = self.world.get_blueprint_library().filter("vehicle.*")
        if not blueprints:
            raise RuntimeError("CARLA world has no vehicle blueprints")
        self.vehicle_blueprint = blueprints[0]
        self._npc_actors = spawn_npc_vehicles(self.world, num_vehicles=10, seed=self.seed)

    def _validate_dt(self, dt):
        if self.sync_mode and dt is not None and (
            not math.isfinite(dt) or
            not math.isclose(dt, self.fixed_delta_seconds, rel_tol=1e-9, abs_tol=1e-12)
        ):
            raise ValueError(
                f"CARLA advances {self.fixed_delta_seconds}s per tick, but dt={dt} was requested"
            )

    def __tick(self, dt=None):
        self._validate_dt(dt)
        if not self.sync_mode:
            return
        with self._tick_lock:
            if self._closed:
                raise RuntimeError("CARLA bridge is closed")
            if self._sync_fault is not None:
                raise RuntimeError("A previous CARLA tick failed; reset the bridge before advancing again")
            # Mark the in-progress frame invalid; readers also acquire this lock.
            self._sync_fault = "CARLA tick has not completed"
            frame_number = self.world.tick()
            try:
                self.__wait_for_sensor_frame(frame_number)
            except Exception:
                self._sync_fault = f"No complete sensor pair for CARLA frame {frame_number}"
                raise
            self._sync_fault = None
            self.__update_camera_position_and_state()

    def __wait_for_sensor_frame(self, frame, timeout=None):
        deadline = time.monotonic() + (self.sensor_timeout if timeout is None else timeout)
        with self._sensor_condition:
            while frame not in self._completed:
                if self._closed:
                    raise RuntimeError("CARLA bridge closed while waiting for sensors")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Missing exact RGB/LiDAR pair for CARLA frame {frame}")
                self._sensor_condition.wait(remaining)
            # A successful tick publishes precisely this frame, not a later callback.
            self._latest_frame = frame
            self._last_delivery = time.monotonic()

    def step(self, dt=None):
        """Script-only advance; normal AVLite execution ticks through control."""
        with self._tick_lock:
            self._ensure_vehicle()
            self.__tick(dt)

    def _record_sensor(self, kind, frame, sensor, capture_pose, generation):
        with self._sensor_condition:
            if self._closed or generation != self._sensor_generation:
                return
            if self._latest_frame is not None and frame <= self._latest_frame:
                return
            row = self._pending.setdefault(frame, {})
            row[kind] = sensor
            if capture_pose is not None:
                row["pose"] = capture_pose
            if "rgb" in row and "lidar" in row:
                if not math.isclose(row["rgb"].stamp, row["lidar"].stamp, rel_tol=0, abs_tol=1e-6):
                    del self._pending[frame]
                    self._sensor_condition.notify_all()
                    return
                self._completed[frame] = (row["rgb"], row["lidar"], row["pose"])
                del self._pending[frame]
                if not self.sync_mode:
                    self._latest_frame = frame
                    self._last_delivery = time.monotonic()
            # Bound memory even when one device never delivers.
            for cache in (self._pending, self._completed):
                while len(cache) > self.sensor_queue_size:
                    del cache[min(cache)]
            self._sensor_condition.notify_all()

    def _on_lidar(self, measurement, generation=None):
        generation = self._sensor_generation if generation is None else generation
        points = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        points[:, 1] *= -1
        # One callback is ONE CARLA instant. Do not accumulate moving scenes.
        sensor = replace(
            self._lidar_mount, points=_readonly(points), stamp=float(measurement.timestamp),
        )
        sensor_to_map = _REFLECTION @ np.asarray(measurement.transform.get_matrix()) @ _REFLECTION
        body_to_map = sensor_to_map @ np.linalg.inv(self._lidar_mount.base_to_sensor)
        self._record_sensor("lidar", int(measurement.frame), sensor, _readonly(body_to_map), generation)

    def _on_rgb(self, image, generation=None):
        generation = self._sensor_generation if generation is None else generation
        raw = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        if (image.width, image.height) != (self._camera_mount.width, self._camera_mount.height):
            raise ValueError("CARLA image resolution no longer matches camera calibration")
        sensor = replace(self._camera_mount, rgb=_readonly(raw[:, :, 2::-1]), stamp=float(image.timestamp))
        self._record_sensor("rgb", int(image.frame), sensor, None, generation)

    def get_sensor_frame(self, agent_id=EGO_AGENT_ID):
        self._require_ego_agent(agent_id, "sensor frame")
        with self._tick_lock:
            if self._closed:
                raise RuntimeError("CARLA bridge is closed")
            if self._sync_fault is not None:
                raise TimeoutError(self._sync_fault)
            with self._sensor_condition:
                number = self._latest_frame
                if number is None:
                    if (not self.sync_mode and self._sensor_started_at is not None and
                            time.monotonic() - self._sensor_started_at > self.max_sensor_age):
                        raise TimeoutError("No matched CARLA sensor pair has arrived since sensor startup")
                    camera, lidar, pose = self._camera_mount, self._lidar_mount, None
                else:
                    if not self.sync_mode and time.monotonic() - self._last_delivery > self.max_sensor_age:
                        raise TimeoutError("Latest matched CARLA sensor pair is stale")
                    camera, lidar, pose = self._completed[number]
                # Fresh wrappers: capability filtering cannot mutate cached snapshots.
                frame = CarlaSensorFrame(
                    cameras={"front": replace(camera)}, lidars={"roof": replace(lidar)},
                    primary_camera_name="front", primary_lidar_name="roof",
                    frame_id="base_link", carla_frame=number,
                    base_to_map=pose,
                )
            return self._apply_world_capability_filter(frame)

    def get_camera_sensor(self, agent_id=EGO_AGENT_ID):
        return self.get_sensor_frame(agent_id).camera

    def get_lidar_sensor(self, agent_id=EGO_AGENT_ID):
        return self.get_sensor_frame(agent_id).lidar

    def get_rgb_image(self, agent_id=EGO_AGENT_ID):
        return self.get_camera_sensor(agent_id).rgb

    def get_depth_image(self, agent_id=EGO_AGENT_ID):
        self._require_ego_agent(agent_id, "depth")
        return None

    def get_lidar_data(self, agent_id=EGO_AGENT_ID):
        return self.get_lidar_sensor(agent_id).points

    def get_camera_intrinsics(self):
        return self._camera_mount.intrinsic

    def get_camera_extrinsics(self):
        """Static optical->body pose. No mutable current-world transform."""
        return self._camera_mount.base_to_sensor

    @staticmethod
    def world_to_camera(frame: CarlaSensorFrame):
        """Projection transform for THIS acquisition, never the current ego pose."""
        if frame.base_to_map is None or frame.camera is None or frame.camera.rgb is None:
            raise ValueError("This frame has no camera acquisition pose/readings")
        return np.linalg.inv(frame.base_to_map @ frame.camera.base_to_sensor)

    @staticmethod
    def _to_carla_transform(x, y, theta, z):
        return carla.Transform(carla.Location(x=x, y=-y, z=z), carla.Rotation(yaw=-math.degrees(theta)))

    @staticmethod
    def _sync_state(state, transform):
        state.x, state.y, state.z = transform.location.x, -transform.location.y, transform.location.z
        state.theta = -math.radians(transform.rotation.yaw)

    def _ensure_vehicle(self):
        if self._closed:
            raise RuntimeError("CARLA bridge is closed")
        if self.vehicle is not None:
            return
        if self.world is None or self.vehicle_blueprint is None:
            raise RuntimeError("CARLA world is not initialized")
        nearest = min(self.spawn_points, key=lambda p:
                      (p.location.x - self.ego_state.x)**2 + (p.location.y + self.ego_state.y)**2,
                      default=None)
        z = nearest.location.z + 0.5 if nearest is not None else max(1.0, self.ego_state.z)
        requested = self._to_carla_transform(self.ego_state.x, self.ego_state.y, self.ego_state.theta, z)
        candidates = [requested] + ([nearest] if nearest is not None else []) + self.spawn_points
        for pose in candidates:
            self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, pose)
            if self.vehicle is not None:
                break
        if self.vehicle is None:
            raise RuntimeError("Failed to spawn ego at requested pose or any fallback")
        self._sync_state(self.ego_state, self.vehicle.get_transform())
        try:
            self.__attach_sensors()
        except Exception:
            self.__destroy_sensors()
            self.vehicle.destroy()
            self.vehicle = None
            raise

    def __attach_sensors(self):
        self._sensor_started_at = time.monotonic()
        library = self.world.get_blueprint_library()
        lidar_bp = library.find("sensor.lidar.ray_cast")
        rotation = 1 / self.fixed_delta_seconds if self.sync_mode else LIDAR_ROTATION_FREQUENCY
        for key, value in {
            "channels": LIDAR_CHANNELS, "range": LIDAR_RANGE,
            "points_per_second": LIDAR_POINTS_PER_SECOND, "rotation_frequency": rotation,
            "upper_fov": LIDAR_UPPER_FOV, "lower_fov": LIDAR_LOWER_FOV, "sensor_tick": 0.0,
        }.items():
            lidar_bp.set_attribute(key, str(value))
        self._lidar_sensor = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(z=LIDAR_Z_OFFSET)), attach_to=self.vehicle,
        )
        self._lidar_mount = replace(self._lidar_mount, sensor_id=str(self._lidar_sensor.id))
        generation = self._sensor_generation
        self._lidar_sensor.listen(lambda data: self._on_lidar(data, generation))
        camera_bp = library.find("sensor.camera.rgb")
        for key, value in {
            "image_size_x": CAMERA_WIDTH, "image_size_y": CAMERA_HEIGHT,
            "fov": CAMERA_FOV, "sensor_tick": 0.0,
        }.items():
            camera_bp.set_attribute(key, str(value))
        self._rgb_sensor = self.world.spawn_actor(
            camera_bp, carla.Transform(
                carla.Location(x=CAM_X, y=CAM_Y, z=CAM_Z),
                carla.Rotation(pitch=CAM_PITCH, yaw=CAM_YAW),
            ), attach_to=self.vehicle,
        )
        self._camera_mount = replace(self._camera_mount, sensor_id=str(self._rgb_sensor.id))
        self._rgb_sensor.listen(lambda data: self._on_rgb(data, generation))

    def _clear_captures(self):
        with self._sensor_condition:
            self._sensor_generation += 1
            self._pending.clear()
            self._completed.clear()
            self._latest_frame = self._last_delivery = self._sync_fault = None
            self._sensor_started_at = None
            self._sensor_condition.notify_all()

    def __destroy_sensors(self):
        # Invalidate before stopping: late callbacks from the old actors are ignored.
        self._clear_captures()
        for attr in ("_lidar_sensor", "_rgb_sensor", "_depth_sensor"):
            sensor = getattr(self, attr)
            setattr(self, attr, None)
            if sensor is not None:
                try:
                    sensor.stop()
                except Exception:
                    log.exception("Could not stop sensor")
                try:
                    sensor.destroy()
                except Exception:
                    log.exception("Could not destroy sensor")
        self._lidar_mount = replace(self._lidar_mount, sensor_id=None)
        self._camera_mount = replace(self._camera_mount, sensor_id=None)

    def control_ego_state(self, cmd: ControlCommand, dt=None):
        self._validate_dt(dt)  # before applying control or advancing the server
        with self._tick_lock:
            self._ensure_vehicle()
            max_acc = self.controller.ego_max_acceleration if self.controller else 10.0
            max_brake = abs(self.controller.ego_min_acceleration) if self.controller else 20.0
            throttle = np.clip(max(0., cmd.acceleration) / max_acc, 0., 1.)
            brake = np.clip(max(0., -cmd.acceleration) / max_brake, 0., 1.)
            reverse = cmd.acceleration < 0 and self.ego_state.velocity < .1
            if reverse:
                throttle, brake = np.clip(abs(cmd.acceleration) / max_acc, 0., 1.), 0.
            self.vehicle.apply_control(carla.VehicleControl(
                throttle=float(throttle), brake=float(brake),
                steer=float(np.clip(-cmd.steer, -1., 1.)), reverse=bool(reverse),
            ))
            self.__tick(dt)
            self.get_ego_state()

    def get_ego_state(self):
        with self._tick_lock:
            self._ensure_vehicle()
            self._sync_state(self.ego_state, self.vehicle.get_transform())
            velocity = self.vehicle.get_velocity()
            self.ego_state.velocity = math.hypot(velocity.x, velocity.y)
            return replace(self.ego_state)

    def teleport_ego(self, x, y, theta=None):
        with self._tick_lock:
            self.ego_state.x, self.ego_state.y = x, y
            if theta is not None:
                self.ego_state.theta = theta
            if self.vehicle is None:
                self._ensure_vehicle()
                return
            z = self.vehicle.get_transform().location.z
            # Reattach to give late pre-teleport callbacks a different generation.
            self.__destroy_sensors()
            self.vehicle.set_transform(self._to_carla_transform(x, y, self.ego_state.theta, z))
            self._sync_state(self.ego_state, self.vehicle.get_transform())
            self.__attach_sensors()

    def spawn_agent(self, agent_state, global_plan=None):
        """Spawn an NPC without replacing the bridge's ego vehicle or sensors."""
        with self._tick_lock:
            actor = self.world.try_spawn_actor(
                self.vehicle_blueprint,
                self._to_carla_transform(agent_state.x, agent_state.y, agent_state.theta, agent_state.z),
            )
            if actor is None:
                raise RuntimeError("Requested NPC spawn pose is occupied")
            self._npc_actors.append(actor)
            actor.set_autopilot(True)

    def get_ground_truth_perception_model(self):
        with self._tick_lock:
            agents = []
            for actor in self.world.get_actors().filter("vehicle.*"):
                if self.vehicle is not None and actor.id == self.vehicle.id:
                    continue
                transform, velocity, bbox = actor.get_transform(), actor.get_velocity(), actor.bounding_box
                agents.append(AgentState(
                    x=transform.location.x, y=-transform.location.y, z=transform.location.z,
                    theta=-math.radians(transform.rotation.yaw),
                    velocity=math.hypot(velocity.x, velocity.y), agent_id=int(actor.id),
                    length=bbox.extent.x*2, width=bbox.extent.y*2,
                ))
            if self.use_static_objects:
                agents.extend(self.get_static_objects())
            return PerceptionModel(ego_vehicle=self.get_ego_state(), agent_vehicles=agents)

    def get_static_objects(self):
        agents = []
        for label in self.static_vehicle_labels:
            for bbox in self.world.get_level_bbs(label):
                agents.append(AgentState(
                    x=bbox.location.x, y=-bbox.location.y, z=bbox.location.z,
                    theta=-math.radians(bbox.rotation.yaw), velocity=0.,
                    agent_id=-(len(agents)+1), length=bbox.extent.x*2, width=bbox.extent.y*2,
                ))
        return agents

    def start_bg_camera_and_state_update(self, interval=.01):
        if self._camera_thread is not None and self._camera_thread.is_alive():
            return
        def follow():
            while not self._stop_event.wait(interval):
                with self._tick_lock:
                    if self._closed:
                        return
                    try:
                        self.__update_camera_position_and_state()
                    except Exception:
                        log.exception("Spectator update failed")
        self._camera_thread = threading.Thread(target=follow, daemon=True, name="carla-spectator")
        self._camera_thread.start()

    def __update_camera_position_and_state(self):
        if self.vehicle is None or self.spectator is None or not self.follow_camera:
            return
        pose = self.vehicle.get_transform()
        yaw = math.radians(pose.rotation.yaw)
        self.spectator.set_transform(carla.Transform(
            carla.Location(x=pose.location.x-self.camera_distance*math.cos(yaw),
                           y=pose.location.y-self.camera_distance*math.sin(yaw),
                           z=pose.location.z+self.camera_height),
            carla.Rotation(pitch=-15, yaw=pose.rotation.yaw),
        ))

    def reset(self):
        with self._tick_lock:
            if self._closed:
                raise RuntimeError("CARLA bridge is closed")
            self.__destroy_sensors()
            self.vehicle = None
            self._npc_actors = []
            self.world = self.client.reload_world(reset_settings=False)
            self.__configure_sync_mode()
            self._prepare_world()
            # Ego respawns lazily after the executor restores its configured start pose.

    def close(self):
        if self._closed:
            return
        self._stop_event.set()
        # Wake a blocked tick before waiting to acquire its lock.
        with self._sensor_condition:
            self._closed = True
            self._sensor_condition.notify_all()
        with self._tick_lock:
            self.__destroy_sensors()
            for actor in [self.vehicle, *self._npc_actors]:
                if actor is not None:
                    try:
                        actor.destroy()
                    except Exception:
                        log.exception("Could not destroy owned actor")
            self.vehicle = None
            self._npc_actors = []
            if self._traffic_manager is not None:
                try:
                    self._traffic_manager.set_synchronous_mode(False)
                except Exception:
                    log.exception("Could not release Traffic Manager synchronous mode")
            if self.world is not None and self._original_settings is not None:
                try:
                    self.world.apply_settings(self._original_settings)
                except Exception:
                    log.exception("Could not restore CARLA world settings")
        if self._camera_thread is not None and self._camera_thread is not threading.current_thread():
            self._camera_thread.join(timeout=2)
        if _ACTIVE_BRIDGES.get(self._owner_key) is self:
            del _ACTIVE_BRIDGES[self._owner_key]


def spawn_npc_vehicles(world, num_vehicles=10, seed=0):
    rng = random.Random(seed)
    blueprints = world.get_blueprint_library().filter("vehicle.*")
    actors = []
    for transform in world.get_map().get_spawn_points()[:num_vehicles]:
        actor = world.try_spawn_actor(rng.choice(blueprints), transform)
        if actor is not None:
            actors.append(actor)
            actor.set_autopilot(True)
    return actors


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
