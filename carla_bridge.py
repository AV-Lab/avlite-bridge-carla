from avlite import (
    AgentState,
    ControlCommand,
    ControlStrategy,
    DepthImage,
    EgoState,
    Lidar,
    LidarCloud,
    PerceptionModel,
    RgbImage,
    SensorFrame,
    StackCapability,
    WorldBridge,
    WorldCapability,
)
from typing import Union
import math
import logging
import numpy as np
import time
import threading
from typing import Optional
log = logging.getLogger(__name__)

# LiDAR sensor defaults
LIDAR_CHANNELS = 32
LIDAR_RANGE = 100.0          # metres
LIDAR_POINTS_PER_SECOND = 500_000
LIDAR_ROTATION_FREQUENCY = 10  # Hz
LIDAR_UPPER_FOV = 10.0
LIDAR_LOWER_FOV = -30.0
LIDAR_Z_OFFSET = 2.4          # sensor height above vehicle origin

try:
    import carla
except ImportError:
    log.error("Carla module not found. Please ensure you have the Carla Python API installed if you need to integrate with Carla.")

class Carla5Bridge(WorldBridge):
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
    ):
        self.supports_ground_truth_detection = True
        self.supports_ground_truth_localization = True
        self.reference_point = reference_point

        self.client = None
        self.world = None
        self.ego_state = ego_state
        self.controller = controller

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
        self._lidar_buffer: Optional[np.ndarray] = None   # (N,4) lidar-frame [x,y,z,intensity]
        # Static lidar mount in the ego body frame: same offset used to attach the sensor.
        mount = np.eye(4)
        mount[2, 3] = LIDAR_Z_OFFSET
        self._lidar_mount = Lidar(base_to_sensor=mount)
        self._rgb_buffer: Optional[np.ndarray] = None      # (H,W,3) uint8
        self._depth_buffer: Optional[np.ndarray] = None    # (H,W) float32 metres
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
            log.info(f"Available maps: {self.client.get_available_maps()}")

            if scene_name not in self.client.get_available_maps():
                raise ValueError(f"Scene {scene_name} not found in available maps.")
            self.world = self.client.load_world(scene_name)
            log.info(f"Connected to Carla at {host}:{port} and loaded scene {scene_name}")

            # Get the spectator to control the camera
            self.spectator = self.world.get_spectator()

            self.spawn_points = self.world.get_map().get_spawn_points()
            log.info(f"Found {len(self.spawn_points)} spawn points in the map")

            spawn_npc_vehicles(self.world, num_vehicles=10)  
            # Initialize vehicle blueprint
            self.__initialize_vehicle_blueprint()
            self.start_bg_camera_and_state_update()

        except Exception as e:
            log.error(f"Failed to connect to Carla: {e}")
            log.error("Make sure the Carla simulator is running on the specified host and port.")


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

    # ------------------------------------------------------------------
    # Handedness: CARLA/UE4 is left-handed (y right, yaw clockwise), AVLite is
    # right-handed (y left, theta counter-clockwise). These two helpers are the
    # only place the sign flip lives.
    # ------------------------------------------------------------------
    @staticmethod
    def _to_carla_transform(x: float, y: float, theta: float, z: float) -> "carla.Transform":
        """AVLite pose (map frame) → carla.Transform."""
        return carla.Transform(
            carla.Location(x=x, y=-y, z=z),
            carla.Rotation(yaw=-math.degrees(theta)),
        )

    @staticmethod
    def _sync_state(state: Union[EgoState, AgentState], transform: "carla.Transform") -> None:
        """Write a carla.Transform into an AVLite state (x, y, theta)."""
        state.x = transform.location.x
        state.y = -transform.location.y
        state.theta = -math.radians(transform.rotation.yaw)

    def _nearest_spawn_point(self, state: Union[EgoState, AgentState]):
        """CARLA spawn point closest to the AVLite pose, or None."""
        if not self.spawn_points:
            return None
        return min(
            self.spawn_points,
            key=lambda p: (p.location.x - state.x) ** 2 + (p.location.y + state.y) ** 2,
        )

    def __spawn_vehicle(self, state: Union[EgoState, AgentState]):
        """Spawn the ego vehicle at the requested AVLite pose.

        The nearest CARLA spawn point is used only for its ground height. If the
        requested pose is blocked, fall back to that spawn point. Either way
        ``state`` is synced to where the vehicle actually landed.
        """
        if not self.world or not self.vehicle_blueprint:
            log.error("Cannot spawn vehicle: world not connected or blueprint not initialized")
            return

        nearest = self._nearest_spawn_point(state)
        z = nearest.location.z + 0.5 if nearest is not None else 1.0
        requested = self._to_carla_transform(state.x, state.y, state.theta, z)
        self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, requested)
        if self.vehicle:
            log.info(f"Spawned ego at requested pose ({state.x:.2f}, {state.y:.2f}, {state.theta:.2f})")
        elif nearest is not None:
            log.warning(
                f"Requested pose ({state.x:.2f}, {state.y:.2f}) is blocked; falling back to nearest spawn point."
            )
            self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, nearest)
            if not self.vehicle:
                for i, spawn_point in enumerate(self.spawn_points):
                    self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, spawn_point)
                    if self.vehicle:
                        log.info(f"Spawned at alternative spawn point {i}")
                        break

        if not self.vehicle:
            log.error("Failed to spawn vehicle at any pose!")
            return
        self._sync_state(state, self.vehicle.get_transform())
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
        lidar_bp.set_attribute('rotation_frequency', str(LIDAR_ROTATION_FREQUENCY))
        lidar_bp.set_attribute('upper_fov', str(LIDAR_UPPER_FOV))
        lidar_bp.set_attribute('lower_fov', str(LIDAR_LOWER_FOV))
        lidar_transform = carla.Transform(carla.Location(z=LIDAR_Z_OFFSET))
        self._lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        self._lidar_sensor.listen(self._on_lidar)
        log.info(f"LiDAR sensor attached ({LIDAR_CHANNELS}ch, {LIDAR_RANGE}m range)")

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
        with self._rgb_lock:
            self._rgb_buffer = None
        with self._depth_lock:
            self._depth_buffer = None

    # ------------------------------------------------------------------
    # Sensor callbacks (run on Carla's sensor thread)
    # ------------------------------------------------------------------
    def _on_lidar(self, measurement):
        """Store carla.LidarMeasurement as an (N,4) array in the lidar's own frame.

        Raw points are already sensor-local [x, y, z, intensity]; only the
        handedness changes (CARLA/UE4 left-handed → AVLite right-handed, negate y).
        The ego pose is never applied here — the stack composes it from its own
        estimate via ``lidar_sensor.to_map``.
        """
        data = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        data[:, 1] *= -1.0
        with self._lidar_lock:
            self._lidar_buffer = data

    # ------------------------------------------------------------------
    # WorldBridge sensor overrides
    # ------------------------------------------------------------------
    def get_lidar_data(self) -> Optional[LidarCloud]:
        """Return latest LiDAR point cloud as (N,4) [x,y,z,intensity] in the lidar frame."""
        with self._lidar_lock:
            return self._lidar_buffer

    def get_lidar_sensor(self) -> Lidar:
        """Static lidar mount in the ego body frame (z = LIDAR_Z_OFFSET)."""
        return self._lidar_mount

    def get_rgb_image(self) -> Optional[RgbImage]:
        with self._rgb_lock:
            return self._rgb_buffer

    def get_depth_image(self) -> Optional[DepthImage]:
        with self._depth_lock:
            return self._depth_buffer

    def get_sensor_frame(self) -> SensorFrame:
        """Return an atomic snapshot of buffered sensor data."""
        with self._rgb_lock:
            rgb = self._rgb_buffer
        with self._depth_lock:
            depth = self._depth_buffer
        with self._lidar_lock:
            lidar = self._lidar_buffer
        return SensorFrame(rgb=rgb, depth=depth, lidar=lidar, lidar_sensor=self._lidar_mount)

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

        # Update self.ego_state from vehicle
        self.get_ego_state()
    

    def teleport_ego(self, x: float, y: float, theta: Optional[float] = None):
        """Move the ego to an AVLite pose; ``theta`` None keeps the current heading."""
        self.ego_state.x = x
        self.ego_state.y = y
        if theta is not None:
            self.ego_state.theta = theta
        if not self.vehicle:
            self.__spawn_vehicle(self.ego_state)  # spawns at ego_state, nothing more to do
            return
        z = self.vehicle.get_transform().location.z
        self.vehicle.set_transform(
            self._to_carla_transform(self.ego_state.x, self.ego_state.y, self.ego_state.theta, z)
        )

    def get_ego_state(self):
        """Read the ego pose and speed back from CARLA into ``ego_state`` (AVLite handedness)."""
        if not self.vehicle:
            self.__spawn_vehicle(self.ego_state)
        velocity = self.vehicle.get_velocity()
        self._sync_state(self.ego_state, self.vehicle.get_transform())
        self.ego_state.velocity = (velocity.x**2 + velocity.y**2) ** 0.5
        log.debug(f"Updated Ego State: x={self.ego_state.x}, y={self.ego_state.y}, theta={self.ego_state.theta}, velocity={self.ego_state.velocity}")
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
        Destroys the current vehicle and reloads the world. The ego is not
        re-spawned here: the executer restores ``ego_state`` to its start pose
        right after this call, and the next tick spawns lazily at that pose.
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
                # Apply a tick to synchronize
                self.world.tick()

                # Reset the simulation to its initial state
                # This is a more thorough reset than just destroying actors
                self.world = self.client.reload_world()

                # Get the spectator again after world reload
                self.spectator = self.world.get_spectator()

                # Refresh spawn points
                self.spawn_points = self.world.get_map().get_spawn_points()

                # Set weather to clear day again
                weather = carla.WeatherParameters.ClearNoon
                self.world.set_weather(weather)

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
    

