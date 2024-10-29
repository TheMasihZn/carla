import carla
import random

import car_manager
import cars
from bridge import CarlaBridge
from cars import Ego
from sensor_manager import SensorManager
from hud import HUD
from agent import Agent
from router import Router
from traffic_light_manager import TrafficLights

random.seed(0)


# a wrapper to link all the realtime data and show in the HUD screen
# this class also synchronizes the timing of updates of each class with every tick
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
        # init the ego vehicle picture window
        self.hud = HUD(_window_size['height'], _window_size['width'])

        # acess to ego and NPCs
        self.car_manager = _car_manager

        # to update and keep track of all the sensors
        self.sensor_manager = SensorManager(_bridge, self.car_manager.ego, _sensor_list, _window_size)

        # to update and keep track of traffic lights
        self.traffic_light_manager = _traffic_light_manager

        # to update the path taken by all the vehicles
        self.router = _router

        # to make decisions about the control
        self.agent = Agent(_traffic_light_manager=self.traffic_light_manager)

    def on_tick(self, _bridge: CarlaBridge):
        # get feedback from the hud screen, so you can read key commands
        # if user closes the window this function returns "break" to signal stopping of algorythm
        if self.hud.window_closed:
            return 'break'

        self.car_manager.on_tick()
        self.router.on_tick(self.car_manager, _bridge)
        self.traffic_light_manager.on_tick(self.router)

        self.agent.on_tick(
            _car_manager=self.car_manager,
            _tl_manager=self.traffic_light_manager,
            _router=self.router,
            _dt=_bridge.get_tick_dt()
        )

        # update the text format you want to show on the HUD
        self.hud.update_text(
            self.sensor_manager.sensors,
            self.car_manager.ego,
            self.traffic_light_manager
        )

        # updates the HUD string with realtime values
        self.hud.render(self.sensor_manager.sensors)

        # triggers the display of camera with HUD overlay
        self.hud.on_tick()

        return ''

    def close(self):
        self.hud.close()
