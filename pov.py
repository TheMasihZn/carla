import carla
import random

import cars
from bridge import CarlaBridge
from cars import Ego
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
            _car_manager: cars.CarManager,
            _sensor_list: list,
            _traffic_light_manager: TrafficLights,
            _window_size: dict
    ):
        self.hud = HUD(_window_size['height'], _window_size['width'])
        self.car_manager = _car_manager
        self.sensor_manager = SensorManager(_bridge, self.car_manager.ego.actor, _sensor_list, _window_size)
        self.traffic_light_manager = _traffic_light_manager
        self.router = _router
        self.agent = Agent(_traffic_light_manager=self.traffic_light_manager)

    # noinspection PyArgumentList
    def on_tick(self, _bridge: CarlaBridge):
        self.car_manager.on_tick(_bridge.map)
        self.router.on_tick(self.car_manager.ego.transform, _bridge)
        self.traffic_light_manager.update_distances(self.router)

        self.hud.update_text(
            self.sensor_manager.sensors,
            self.car_manager.ego,
            self.traffic_light_manager
        )
        self.hud.on_tick()

        if self.hud.window_closed:
            return 'break'

        self.agent.on_tick(
            _map=_bridge.map,
            _raw_npc_list=self.car_manager.npc_list,
            _router=self.router,
            _tl_manager=self.traffic_light_manager,
            _ego=self.car_manager.ego,
            _destination=self.router.next_destination(),
            _debug_bridge=_bridge
        )

        self.hud.render(self.sensor_manager.sensors)

        return ''

    def close(self):
        self.hud.close()
