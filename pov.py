import carla
import bridge
import random
from sensor_manager import SensorManager
from hud import HUD
from agent import Agent
from router import Router
from controller import VehiclePIDController
from traffic_light_manager import TrafficLights

random.seed(0)


class POV(object):
    def __init__(
            self,
            _bridge: bridge.CarlaBridge,
            _spawn_transform: carla.Transform,
            _router: Router,
            _player: carla.Vehicle,
            _sensor_list: list,
            _traffic_light_manager: TrafficLights,
            _window_size: dict
    ):
        self.hud = HUD(_window_size['height'], _window_size['width'])

        self.player = _player
        self.sensor_manager = SensorManager(_bridge, self.player, _sensor_list, _window_size)
        self.traffic_light_manager = _traffic_light_manager
        self.router = _router
        self.agent = Agent(_npc_list=_bridge.npc_list, _traffic_light_manager=self.traffic_light_manager)
        self.controller = VehiclePIDController(
            self.player,
            _bridge=_bridge,
            offset=0.0,
            max_throttle=1.0,
            max_brake=1.0,
            max_steering=0.8
        )

    # noinspection PyArgumentList
    def on_tick(self, _bridge: bridge.CarlaBridge):
        self.hud.set_text_for_tick(
            self.sensor_manager.sensors,
            self.player.get_transform(),
            self.player.get_velocity(),
            self.player.get_control(),
            self.traffic_light_manager
        )
        self.hud.on_tick()
        if self.hud.stop_signal:
            return 'break'

        self.router.on_tick(self.player.get_transform())
        next_dest = self.router.next_destination()

        if not next_dest:
            return 'break'

        next_3_waypoints = [
            _bridge.map.get_waypoint(
                step['transform'].location
            ) for step in self.router.next_n(n=3)
        ]
        distance_to_stop_in = False
        # distance_to_stop_in = self.agent.should_stop(
        #     next_3_waypoints
        # )

        control = self.controller.run_step(
            40.0,
            _bridge.map.get_waypoint(
                next_dest['transform'].location
            ),
            distance_to_stop_in
        )
        self.player.apply_control(control)

        self.hud.render(self.sensor_manager.sensors)

        return ''

    def close(self):
        self.hud.close()