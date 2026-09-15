"""Regression tests with a deterministic fake CARLA server; no network/GUI."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest


class Transform:
    def __init__(self, location=None, rotation=None, matrix=None):
        self.location = location or NS(x=0., y=0., z=0.)
        self.rotation = rotation or NS(yaw=0., pitch=0., roll=0.)
        self.matrix = matrix

    def get_matrix(self):
        if self.matrix is not None:
            return self.matrix
        cy, sy = math.cos(math.radians(self.rotation.yaw)), math.sin(math.radians(self.rotation.yaw))
        cp, sp = math.cos(math.radians(self.rotation.pitch)), math.sin(math.radians(self.rotation.pitch))
        cr, sr = math.cos(math.radians(self.rotation.roll)), math.sin(math.radians(self.rotation.roll))
        result = np.eye(4)
        result[:3, :3] = [
            [cp*cy, cy*sp*sr-sy*cr, -cy*sp*cr-sy*sr],
            [cp*sy, sy*sp*sr+cy*cr, -sy*sp*cr+cy*sr],
            [sp, -cp*sr, cp*cr],
        ]
        result[:3, 3] = [self.location.x, self.location.y, self.location.z]
        return result


class Blueprint:
    def __init__(self, kind):
        self.kind, self.attrs = kind, {}

    def set_attribute(self, key, value):
        self.attrs[key] = value


class Actor:
    def __init__(self, world, blueprint, transform, parent=None):
        self.world, self.blueprint, self.transform, self.parent = world, blueprint, transform, parent
        self.id = len(world.actors) + 1
        self.callback = None
        self.destroyed = self.stopped = False
        self.controls = []
        self.bounding_box = NS(extent=NS(x=2., y=1.))

    def get_transform(self):
        return self.transform

    def get_velocity(self):
        return NS(x=1., y=0., z=0.)

    def set_transform(self, transform):
        self.transform = transform

    def apply_control(self, command):
        self.controls.append(command)

    def set_autopilot(self, value):
        self.autopilot = value

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stopped = True

    def destroy(self):
        self.destroyed = True


class World:
    def __init__(self, settings=None):
        self.settings = settings or NS(synchronous_mode=False, fixed_delta_seconds=None,
            substepping=True, max_substep_delta_time=.01, max_substeps=10)
        self.actors, self.blueprints, self.spawn_points = [], {}, []
        self.frame, self.elapsed, self.tick_hook = 0, 0., None
        self.spectator = NS(set_transform=lambda transform: None)

    def get_settings(self):
        return copy.copy(self.settings)

    def apply_settings(self, settings):
        self.settings = copy.copy(settings)

    def get_map(self):
        return NS(get_spawn_points=lambda: self.spawn_points)

    def get_blueprint_library(self):
        def find(kind):
            self.blueprints.setdefault(kind, Blueprint(kind))
            return self.blueprints[kind]
        return NS(find=find, filter=lambda pattern: [find("vehicle.test")])

    def get_spectator(self):
        return self.spectator

    def spawn_actor(self, blueprint, transform, attach_to=None):
        actor = Actor(self, blueprint, transform, attach_to)
        self.actors.append(actor)
        return actor

    try_spawn_actor = spawn_actor

    def get_actors(self):
        return NS(filter=lambda pattern: [
            a for a in self.actors if a.blueprint.kind.startswith("vehicle.") and not a.destroyed
        ])

    def tick(self):
        self.frame += 1
        self.elapsed += self.settings.fixed_delta_seconds
        if self.tick_hook:
            self.tick_hook(self)
        else:
            self.deliver("lidar")
            self.deliver("rgb")
        return self.frame

    def deliver(self, kind, frame=None, stamp=None, x=3., y=2.):
        target = "sensor.lidar.ray_cast" if kind == "lidar" else "sensor.camera.rgb"
        for actor in self.actors:
            if actor.blueprint.kind != target or actor.destroyed:
                continue
            matrix = np.asarray(actor.parent.transform.get_matrix()) @ actor.transform.get_matrix()
            raw = (np.array([[x, y, 0., 1]], np.float32).tobytes() if kind == "lidar"
                   else np.broadcast_to(np.array([1, 2, 3, 255], np.uint8), (720, 1280, 4)).tobytes())
            data = NS(frame=self.frame if frame is None else frame,
                      timestamp=self.elapsed if stamp is None else stamp, raw_data=raw,
                      transform=Transform(matrix=matrix), width=1280, height=720)
            actor.callback(data)


@pytest.fixture
def module(monkeypatch):
    fake = ModuleType("carla")
    fake.Location = lambda x=0., y=0., z=0.: NS(x=x, y=y, z=z)
    fake.Rotation = lambda yaw=0., pitch=0., roll=0.: NS(yaw=yaw, pitch=pitch, roll=roll)
    fake.Transform, fake.VehicleControl = Transform, NS
    fake.CityObjectLabel = NS(**{n:n for n in ("Car", "Truck", "Bus", "Motorcycle", "Bicycle")})
    class Client:
        def __init__(self, *args):
            self.world, self.reloads = World(), []
            self.tm = NS(set_synchronous_mode=lambda mode: None, set_random_device_seed=lambda seed: None)
        def set_timeout(self, timeout): pass
        def load_world(self, name): return self.world
        def get_trafficmanager(self): return self.tm
        def reload_world(self, reset_settings=True):
            self.reloads.append(reset_settings)
            self.world = World(None if reset_settings else self.world.get_settings())
            return self.world
    fake.Client = Client
    monkeypatch.setitem(sys.modules, "carla", fake)
    root = Path(__file__).resolve().parents[1]
    pkg = ModuleType("carla_bridge_test")
    pkg.__path__ = [str(root)]
    monkeypatch.setitem(sys.modules, pkg.__name__, pkg)
    for name in ("settings", "carla_bridge"):
        fullname = f"{pkg.__name__}.{name}"
        spec = importlib.util.spec_from_file_location(fullname, root / f"{name}.py")
        mod = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, fullname, mod)
        spec.loader.exec_module(mod)
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c41_world_capabilities", None)
    monkeypatch.setattr(ExecutionSettings, "c40_pace_sim", True)
    monkeypatch.setattr(ExecutionSettings, "c40_sim_dt", .05)
    return mod


@pytest.fixture
def bridge(module):
    bridge = module.Carla4Bridge(module.EgoState(), sync_mode=True)
    bridge.sensor_timeout = .2
    yield bridge
    bridge.close()


def test_camera_and_lidar_attached_and_full_sweep(bridge):
    bridge.step(.05)
    assert bridge.world.blueprints["sensor.lidar.ray_cast"].attrs["rotation_frequency"] == "20.0"
    assert bridge.world.blueprints["sensor.camera.rgb"].attrs["sensor_tick"] == "0.0"
    frame = bridge.get_sensor_frame()
    assert frame.carla_frame == 1
    assert frame.camera.stamp == frame.lidar.stamp == .05
    np.testing.assert_equal(frame.camera.rgb[0,0], [3,2,1])
    np.testing.assert_equal(frame.lidar.points, [[3,-2,0,1]])
    assert frame.get_lidar(str(bridge._lidar_sensor.id)) is frame.lidar
    assert frame.get_camera("front") is frame.camera
    assert frame.frame_id == "base_link"


def test_no_tick_from_getters_and_empty_readings_keep_mounts(bridge):
    frame = bridge.get_sensor_frame()
    assert frame.lidar.points is frame.camera.rgb is None
    for _ in range(3):
        bridge.get_lidar_data()
        bridge.get_rgb_image()
    assert bridge.world.frame == 0


def test_dt_mismatch_fails_before_any_server_step(bridge, module):
    with pytest.raises(ValueError, match="dt=0.01"):
        bridge.control_ego_state(module.ControlCommand(), dt=.01)
    assert bridge.world.frame == 0 and bridge.vehicle is None
    for _ in range(10):
        bridge.control_ego_state(module.ControlCommand(), dt=.05)
    assert bridge.world.frame == 10
    assert bridge.world.elapsed == pytest.approx(.5)


def test_timeout_is_latched_and_never_returns_stale_pair(bridge):
    bridge.step(.05)
    bridge.world.tick_hook = lambda world: world.deliver("lidar")
    with pytest.raises(TimeoutError, match="frame 2"):
        bridge.step(.05)
    with pytest.raises(TimeoutError):
        bridge.get_sensor_frame()
    with pytest.raises(RuntimeError, match="reset"):
        bridge.step(.05)
    assert bridge.world.frame == 2
    bridge.reset()
    bridge.step(.05)
    assert bridge.get_sensor_frame().carla_frame == 1


def test_concurrent_reader_cannot_observe_in_progress_pair(bridge):
    bridge.step(.05)
    lidar_ready, deliver_rgb, reader_started, reader_done = (threading.Event() for _ in range(4))
    errors, results = [], []
    bridge.sensor_timeout = 2
    def partial(world):
        world.deliver("lidar")
        lidar_ready.set()
        assert deliver_rgb.wait(1)
        world.deliver("rgb")
    bridge.world.tick_hook = partial
    def advance():
        try: bridge.step(.05)
        except BaseException as exc: errors.append(exc)
    def read():
        reader_started.set()
        try: results.append(bridge.get_sensor_frame())
        except BaseException as exc: errors.append(exc)
        finally: reader_done.set()
    ticker = threading.Thread(target=advance)
    reader = threading.Thread(target=read)
    ticker.start()
    try:
        assert lidar_ready.wait(1)
        reader.start()
        assert reader_started.wait(1)
        assert not reader_done.wait(.03)
    finally:
        deliver_rgb.set()
        ticker.join(2)
        if reader.ident is not None: reader.join(2)
    assert not errors and not ticker.is_alive() and not reader.is_alive()
    assert results[0].carla_frame == 2
    assert results[0].camera.stamp == results[0].lidar.stamp == .1


def test_async_pairs_exact_frames_and_does_not_accumulate(bridge):
    bridge._ensure_vehicle()
    bridge.sync_mode = False
    world = bridge.world
    world.deliver("rgb", frame=10, stamp=1.)
    world.deliver("lidar", frame=11, stamp=1.1)
    assert bridge.get_sensor_frame().carla_frame is None
    world.deliver("lidar", frame=10, stamp=1.)
    first = bridge.get_sensor_frame()
    assert first.carla_frame == 10 and len(first.lidar.points) == 1
    world.deliver("rgb", frame=11, stamp=1.1)
    second = bridge.get_sensor_frame()
    assert second.carla_frame == 11 and len(second.lidar.points) == 1
    assert first.camera.stamp == first.lidar.stamp == 1.
    assert second.camera.stamp == second.lidar.stamp == 1.1
    assert first.stamp is None  # no guessed assembly/acquisition timestamp


def test_timestamp_mismatch_does_not_publish(bridge):
    bridge._ensure_vehicle()
    bridge.sync_mode = False
    bridge.world.deliver("rgb", frame=10, stamp=1.)
    bridge.world.deliver("lidar", frame=10, stamp=9.)
    assert bridge.get_sensor_frame().carla_frame is None


def test_buffers_bounded_and_async_stale_pair_rejected(bridge):
    bridge._ensure_vehicle()
    bridge.sync_mode = False
    bridge.sensor_queue_size = 3
    for i in range(10):
        bridge.world.deliver("lidar", frame=i, stamp=i*.05)
    assert len(bridge._pending) == 3
    bridge.world.deliver("rgb", frame=9, stamp=.45)
    bridge._last_delivery = time.monotonic() - bridge.max_sensor_age - 1
    with pytest.raises(TimeoutError, match="stale"):
        bridge.get_sensor_frame()


def test_captured_projection_not_changed_by_later_ego_pose(bridge):
    bridge.step(.05)
    frame = bridge.get_sensor_frame()
    old_transform = bridge.world_to_camera(frame).copy()
    bridge.ego_state.x += 10
    bridge.vehicle.transform.location.x += 10
    bridge.step(.05)
    np.testing.assert_equal(bridge.world_to_camera(frame), old_transform)
    assert not np.array_equal(bridge.world_to_camera(bridge.get_sensor_frame()), old_transform)
    assert frame.base_to_map[2,3] == pytest.approx(1.)
    assert bridge.ego_state.z == pytest.approx(1.)


def test_filter_and_consumer_changes_do_not_mutate_cached_wrappers(bridge, monkeypatch):
    bridge.step(.05)
    from avlite.c40_execution.c49_settings import ExecutionSettings
    first = bridge.get_sensor_frame()
    first.lidar.points = None
    first.camera.rgb = None
    monkeypatch.setattr(ExecutionSettings, "c41_world_capabilities", [])
    assert bridge.get_sensor_frame().lidar.points is None
    monkeypatch.setattr(ExecutionSettings, "c41_world_capabilities", None)
    second = bridge.get_sensor_frame()
    assert second.lidar.points is not None and second.camera.rgb is not None
    with pytest.raises(ValueError):
        second.lidar.points[0,0] = 50


def test_old_callback_after_reset_cannot_poison_new_episode(bridge):
    bridge.step(.05)
    old_generation = bridge._sensor_generation
    old = bridge.get_sensor_frame()
    bridge.reset()
    bridge._record_sensor("rgb", 1000, old.camera, None, old_generation)
    bridge._record_sensor("lidar", 1000, old.lidar, old.base_to_map, old_generation)
    assert not bridge._pending and not bridge._completed
    assert bridge.client.reloads == [False, False]
    bridge.step(.05)
    assert bridge.get_sensor_frame().carla_frame == 1


def test_spawn_npc_does_not_replace_ego(bridge, module):
    bridge.step(.05)
    ego, lidar, rgb = bridge.vehicle, bridge._lidar_sensor, bridge._rgb_sensor
    bridge.spawn_agent(module.AgentState(x=10, y=5, z=1), global_plan=None)
    assert bridge.vehicle is ego and bridge._lidar_sensor is lidar and bridge._rgb_sensor is rgb
    assert len(bridge._npc_actors) == 1


def test_brake_is_nonnegative_and_clamped(bridge, module):
    bridge.ego_state.velocity = 5.
    bridge.control_ego_state(module.ControlCommand(acceleration=-100, steer=3), .05)
    command = bridge.vehicle.controls[-1]
    assert command.brake == 1. and command.throttle == 0. and command.steer == -1.


def test_teleport_preserves_unspecified_heading_and_invalidates_sensor_data(bridge):
    bridge.step(.05)
    bridge.ego_state.theta = .7
    bridge.teleport_ego(3,4)
    assert bridge.ego_state.theta == pytest.approx(.7)
    assert bridge.vehicle.transform.location.y == -4
    assert bridge.get_sensor_frame().carla_frame is None
    bridge.teleport_ego(3,4,theta=0.)
    assert bridge.ego_state.theta == 0.


def test_close_stops_owned_sensors_thread_and_restores_settings(bridge):
    bridge.step(.05)
    actors = [bridge.vehicle, bridge._lidar_sensor, bridge._rgb_sensor]
    bridge.start_bg_camera_and_state_update()
    bridge.close()
    assert all(a.destroyed for a in actors)
    assert not bridge._camera_thread.is_alive()
    assert not bridge.world.settings.synchronous_mode
    with pytest.raises(RuntimeError, match="closed"):
        bridge.get_sensor_frame()


@pytest.mark.parametrize("sync_mode", [None, True])
@pytest.mark.parametrize("dt", [.01, .02, .05])
def test_sync_step_comes_from_avlite(module, monkeypatch, sync_mode, dt):
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c40_sim_dt", dt)
    bridge = module.Carla4Bridge(module.EgoState(), sync_mode=sync_mode)
    try:
        assert bridge.fixed_delta_seconds == dt
        assert bridge.world.settings.fixed_delta_seconds == dt
        bridge.step()
        assert bridge.get_sensor_frame().lidar.stamp == pytest.approx(dt)
        assert float(bridge.world.blueprints["sensor.lidar.ray_cast"].attrs[
            "rotation_frequency"]) == pytest.approx(1 / dt)
        with pytest.raises(ValueError, match="per tick"):
            bridge.step(dt * 2)
        assert bridge.world.elapsed == pytest.approx(dt)
    finally:
        bridge.close()


@pytest.mark.parametrize("sync_mode", [None, True])
def test_sync_requires_avlite_pacing_before_connecting(module, monkeypatch, sync_mode):
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c40_pace_sim", False)
    def unexpected_client(*args):
        pytest.fail("Invalid timing must be rejected before connecting to CARLA")
    monkeypatch.setattr(module.carla, "Client", unexpected_client)
    with pytest.raises(ValueError, match="c40_pace_sim"):
        module.Carla4Bridge(module.EgoState(), sync_mode=sync_mode)


@pytest.mark.parametrize("dt", [0., -.01, float("nan"), float("inf")])
def test_sync_rejects_invalid_avlite_duration_before_connecting(module, monkeypatch, dt):
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c40_sim_dt", dt)
    def unexpected_client(*args):
        pytest.fail("Invalid timing must be rejected before connecting to CARLA")
    monkeypatch.setattr(module.carla, "Client", unexpected_client)
    with pytest.raises(ValueError, match="c40_sim_dt must be finite and positive"):
        module.Carla4Bridge(module.EgoState(), sync_mode=True)


def test_async_does_not_require_avlite_fixed_timing(module, monkeypatch):
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c40_pace_sim", False)
    monkeypatch.setattr(ExecutionSettings, "c40_sim_dt", float("nan"))
    monkeypatch.setattr(module.Carla4Bridge, "start_bg_camera_and_state_update", lambda self: None)
    bridge = module.Carla4Bridge(module.EgoState(), sync_mode=False)
    try:
        assert bridge.fixed_delta_seconds is None
        assert bridge.world.settings.fixed_delta_seconds is None
        assert not bridge.world.settings.synchronous_mode
    finally:
        bridge.close()


def test_non_ego_sensor_request_is_explicitly_rejected(bridge):
    with pytest.raises(NotImplementedError):
        bridge.get_sensor_frame(agent_id=42)


def test_mount_is_proper_optical_rotation(bridge):
    matrix = bridge._camera_mount.base_to_sensor
    assert np.linalg.det(matrix[:3,:3]) == pytest.approx(1.)
    np.testing.assert_allclose(matrix[:3,:3] @ [0,0,1], [1,0,0])
    np.testing.assert_allclose(matrix[:3,:3] @ [1,0,0], [0,-1,0])


def test_new_bridge_closes_previous_owner(module):
    first = module.Carla4Bridge(module.EgoState(), sync_mode=True)
    first.step(.05)
    first.start_bg_camera_and_state_update()
    second = module.Carla4Bridge(module.EgoState(), sync_mode=True)
    try:
        assert first._closed and not first._camera_thread.is_alive()
        assert module._ACTIVE_BRIDGES[("localhost", 2000)] is second
    finally:
        second.close()


def test_real_sync_executor_clock_matches_fake_carla(bridge, module, monkeypatch):
    from avlite.c40_execution.c44_sync_executer import SyncExecuter
    from avlite.c40_execution.c49_settings import ExecutionSettings
    monkeypatch.setattr(ExecutionSettings, "c41_world_stack_capabilities", None)
    controller = NS(
        world_requirements=frozenset(), stack_requirements=frozenset(),
        stack_capabilities=frozenset({module.StackCapability.CONTROL}),
        control=lambda *args, **kwargs: module.ControlCommand(),
        reset=lambda: None,
    )
    executor = SyncExecuter(
        perception_model=module.PerceptionModel(), world=bridge, controller=controller,
    )
    for _ in range(10):
        executor.step(sim_dt=.05, control_dt=.05, pace_sim=True, pace_control=False,
                      call_perceive=False, call_replan=False, call_localize=False)
    assert executor.elapsed_sim_time == pytest.approx(.5)
    assert bridge.world.elapsed == pytest.approx(executor.elapsed_sim_time)


def test_callback_during_close_unblocks_tick_without_deadlock(bridge):
    bridge._ensure_vehicle()
    started = threading.Event()
    bridge.world.tick_hook = lambda world: started.set()
    bridge.sensor_timeout = 5
    errors = []
    def advance():
        try: bridge.step(.05)
        except (RuntimeError, TimeoutError) as exc: errors.append(exc)
    worker = threading.Thread(target=advance)
    worker.start()
    assert started.wait(1)
    bridge.close()
    worker.join(1)
    assert not worker.is_alive() and errors


def test_async_missing_initial_camera_eventually_fails(bridge):
    bridge._ensure_vehicle()
    bridge.sync_mode = False
    bridge._sensor_started_at = time.monotonic() - bridge.max_sensor_age - 1
    bridge.world.deliver("lidar", frame=1, stamp=.05)
    with pytest.raises(TimeoutError, match="startup"):
        bridge.get_sensor_frame()


def test_roll_pitch_capture_transform_is_preserved(bridge):
    bridge._ensure_vehicle()
    bridge.vehicle.transform.rotation = NS(yaw=20., pitch=10., roll=5.)
    bridge.step(.05)
    frame = bridge.get_sensor_frame()
    expected = module_reflection = np.diag([1.,-1.,1.,1.])
    expected = module_reflection @ bridge.vehicle.transform.get_matrix() @ module_reflection
    np.testing.assert_allclose(frame.base_to_map, expected, atol=1e-12)
    np.testing.assert_allclose(bridge.world_to_camera(frame) @ frame.base_to_map @ frame.camera.base_to_sensor, np.eye(4), atol=1e-12)
