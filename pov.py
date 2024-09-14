import carla
import bridge
import random
from sensor_manager import SensorManager
from hud import HUD
from agent import Agent
from router import Router
from controller import VehiclePIDController

random.seed(0)


class WorldPOV(object):
    def __init__(
            self,
            _bridge: bridge.CarlaBridge,
            _spawn_transform: carla.Transform,
            size: dict
    ):
        self.hud = HUD(size['height'], size['width'])

        self.player: carla.Vehicle = self.__spawn_hero(
            _bridge,
            _spawn_transform
        )
        self.sensor_manager = SensorManager(_bridge, self.player, size)

        self.router = Router(_bridge)
        self.agent = Agent(self.player, _bridge)
        self.controller = VehiclePIDController(
            self.player,
            _bridge=_bridge,
            args_lateral={
                'K_P': 1.95,
                'K_I': 0.05,
                'K_D': 0.2,
                'dt': 0.0
            },
            args_longitudinal={
                'K_P': 1.0,
                'K_I': 0.05,
                'K_D': 0.0,
                'dt': 0.0
            },
            offset=0.0,
            max_throttle=1.0,
            max_brake=1.0,
            max_steering=0.8
        )

    def __spawn_hero(
            self,
            _bridge: bridge.CarlaBridge,
            _spawn_point: carla.Transform
    ) -> carla.Actor:
        if self:
            pass
        hero_bp = random.choice(_bridge.vehicle_blueprints)
        hero_bp.set_attribute('role_name', 'hero')

        hero = _bridge.spawn_actor(hero_bp, _spawn_point)

        physics_control = hero.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        hero.apply_physics_control(physics_control)

        return hero

    # noinspection PyArgumentList
    def on_tick(self, _bridge: bridge.CarlaBridge):
        self.hud.set_text_for_tick(
            self.sensor_manager.sensors,
            self.player.get_transform(),
            self.player.get_velocity(),
            self.player.get_control(),
            _bridge.traffic_lights.targets
        )

        if self.hud.return_key_pressed():
            return 'break'

        self.router.on_tick(self.player.get_transform())
        next_dest = self.router.next_destination()
        if not next_dest:
            return 'break'

        control = None
        hazard_distance = self.agent.detect_hazard(
            next_dest['transform'].location
        )

        traffic_light_distance = self.agent.traffic_light_detected(self.player)

        if not control:
            control = self.controller.run_step(
                40.0,
                _bridge.map.get_waypoint(
                    next_dest['transform'].location
                ),
                hazard_distance + traffic_light_distance
            )
        self.player.apply_control(control)

        self.hud.render(self.sensor_manager.sensors)

        return ''
