import carla

import bridge
import cars
import router
import traffic_light_manager
from calculation_delegate import location_equal
from controller import PIDController
import math


class Agent(object):
    # noinspection PyArgumentList
    def __init__(
            self,
            _traffic_light_manager
    ):
        self.max_speed = 40
        self.max_steer = 0.8
        self.safe_distance = 5.0

        self.target_speed = self.max_speed

        self.traffic_lights: traffic_light_manager.TrafficLights = _traffic_light_manager

        # self.done_once = False
        # self.start_dist = None
        # self.stop_dist = None
        # self.max_speed = 0.0

        self.pid = PIDController(
            # max_steering=0.8
        )

    def on_tick(
            self,

            _map: carla.Map,
            _router: router.Router,
            _npc_list: list,

            _tl_manager: traffic_light_manager.TrafficLights,
            _ego: cars.Ego,
            _destination: carla.Location,

            _debug_bridge: bridge.CarlaBridge = None
    ):
        should_break = False

        relevant_npc_list = [n for n in _npc_list if n.distance_ego_to_car > 0]
        sorted(
            relevant_npc_list,
            key=lambda _npc: _npc.distance_ego_to_car
        )
        if len(relevant_npc_list) > 0:
            npc: cars.NPC = relevant_npc_list[0]
            if npc.distance_ego_to_car < self.safe_distance:
                should_break = True

            elif npc.distance_ego_to_car < 2 * self.safe_distance:
                self.target_speed = npc.speed

            elif npc.distance_ego_to_car > 6 * self.safe_distance:
                self.target_speed = self.max_speed

        # if self.traffic_lights.targets[0].get_state() == carla.TrafficLightState.Red:
        #     if location_equal(_ego.location, self.traffic_lights.targets[0].get_location(), 7 * self.safe_distance):
        #         should_break = True

        control = self.pid.get_new_control(
            _previous_control=_ego.control,
            _should_stop=should_break,
            _target_speed=self.target_speed,
            _current_speed=_ego.speed,
            _dest=_destination.location,
            _now_at=_ego.location,
            _forward_v=_ego.forward,
        )
        _ego.inject_control(control)
