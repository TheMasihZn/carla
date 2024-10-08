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

        self.traffic_lights = _traffic_light_manager

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

        # d_to_tl = _tl_manager.distance_to_targets[0]
        # d = 0.0
        # should_stop_in = (
        #         (self.max_speed
        #          / (
        #                  _car.damping_rate_zero_throttle_clutch_engaged -
        #                  _car.damping_rate_full_throttle -
        #                  _car.drag
        #          )
        #          ) +
        #         _car.bounding_box.extent.x / 2)
        #
        # control = _car.control
        # if self.max_speed < _car.speed:
        #     self.max_speed = _car.speed
        #
        # if not self.done_once:
        #
        #     if _car.speed / 3.6 <= 10.0:
        #         control.throttle = 1.0
        #         control.brake = 0.0
        #     else:
        #         if not self.start_dist:
        #             self.start_dist = _car.location
        #         control.throttle = 0.0
        #         control.brake = 0.0
        #         self.done_once = True
        # else:
        #     if _car.speed == 0.0:
        #         if not self.stop_dist:
        #             self.stop_dist = _car.location
        #             d = math.sqrt(
        #                 (self.start_dist.x - self.stop_dist.x) ** 2 + (self.start_dist.y - self.stop_dist.y) ** 2)

        # should_stop = False
        # if d_to_tl < self.safe_distance:
        #     control.throttle = 0
        #     control.brake = 1
        #     return control
        #
        # elif d_to_tl < self.safe_distance * 4:
        #     should_stop = True
        #

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
