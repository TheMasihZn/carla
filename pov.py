import carla
import random

from bridge import CarlaBridge
from cars import Car
from sensor_manager import SensorManager
from hud import HUD
from agent import Agent
from router import Router
from traffic_light_manager import TrafficLights

random.seed(0)


class POV(object):
    def __init__(
            self,
            _bridge: CarlaBridge,
            _spawn_transform: carla.Transform,
            _router: Router,
            _car: Car,
            _sensor_list: list,
            _traffic_light_manager: TrafficLights,
            _window_size: dict
    ):
        self.hud = HUD(_window_size['height'], _window_size['width'])
        self.car = _car
        self.sensor_manager = SensorManager(_bridge, self.car, _sensor_list, _window_size)
        self.traffic_light_manager = _traffic_light_manager
        self.router = _router
        self.relevant_npc_list = []
        self.agent = Agent(_traffic_light_manager=self.traffic_light_manager)

    # noinspection PyArgumentList
    def on_tick(self, _bridge: CarlaBridge):
        self.car.update_parameters()
        self.router.on_tick(self.car.transform, _bridge)
        self.traffic_light_manager.update_distances(self.router)

        self.hud.update_text(
            self.sensor_manager.sensors,
            self.car,
            self.traffic_light_manager
        )
        self.hud.on_tick()

        if self.hud.window_closed:
            return 'break'

        self.agent.on_tick(
            _map=_bridge.map,
            _raw_npc_list=_bridge.npc_list,
            _router=self.router,
            _tl_manager=self.traffic_light_manager,
            _car=self.car,
            _destination=self.router.next_destination()
        )

        self.hud.render(self.sensor_manager.sensors)

        return ''

    def close(self):
        self.hud.close()
