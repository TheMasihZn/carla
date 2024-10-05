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
        self.agent = Agent(
            _relevant_npc_list=self.relevant_npc_list,
            _traffic_light_manager=self.traffic_light_manager
        )

    # noinspection PyArgumentList
    def on_tick(self, _bridge: CarlaBridge):
        self.car.update_parameters()
        self.router.on_tick(self.car.transform, _bridge)
        self.traffic_light_manager.update_distances(self.router)
        self.__update_relevant_npc_list()

        self.hud.update_text(
            self.sensor_manager.sensors,
            self.car,
            self.traffic_light_manager
        )
        self.hud.on_tick()

        if self.hud.window_closed:
            return 'break'

        self.agent.on_tick(
            _car=self.car,
            _router=self.router,
            _tl_manager=self.traffic_light_manager,
            _destination=self.router.next_destination()
        )

        self.hud.render(self.sensor_manager.sensors)

        return ''

    def __update_relevant_npc_list(self, _bridge: CarlaBridge):
        self.relevant_npc_list = []
        self.npc_route_id = {}
        for npc in _bridge.npc_list:
            npc_location = npc.get_location()
            wp = _bridge.map.get_waypoint(npc_location)
            if (wp.road_id, wp.lane_id) in self.router.road_lane_pairs:
                self.relevant_npc_list.append(npc)
                self.npc_route_id[npc] = self.router.get_i_in_path(npc_location)

    def close(self):
        self.hud.close()
