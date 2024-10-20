import carla
import random

import car_manager
import cars
from bridge import CarlaBridge
from cars import Ego
from sensor_manager import SensorManager
from hud import HUD
from mpc_agent import MPCAgent
from router import Router
from traffic_light_manager import TrafficLights

random.seed(0)


class POV(object):
    def __init__(
            self,
            _bridge: CarlaBridge,
            _spawn_transform: carla.Transform,
            _router: Router,
            _car_manager: car_manager.CarManager,
            _sensor_list: list,
            _traffic_light_manager: TrafficLights,
            _window_size: dict
    ):
        self.hud = HUD(_window_size['height'], _window_size['width'])
        self.car_manager = _car_manager
        self.sensor_manager = SensorManager(_bridge, self.car_manager.ego, _sensor_list, _window_size)
        self.traffic_light_manager = _traffic_light_manager
        self.router = _router
        self.agent = MPCAgent(_traffic_light_manager=self.traffic_light_manager)

    def on_tick(self, _bridge: CarlaBridge):
        self.car_manager.on_tick()
        self.router.on_tick(self.car_manager, _bridge)
        self.traffic_light_manager.on_tick(self.router)

        if self.hud.window_closed:
            return 'break'

        self.agent.on_tick(
            _car_manager=self.car_manager.npc_list,
            _tl_manager=self.traffic_light_manager,
            _router=self.router,
            _debug_bridge=_bridge
        )

        self.hud.update_text(
            self.sensor_manager.sensors,
            self.car_manager.ego,
            self.traffic_light_manager
        )
        self.hud.render(self.sensor_manager.sensors)
        self.hud.on_tick()

        return ''

    def close(self):
        self.hud.close()
