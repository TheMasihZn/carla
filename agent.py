import carla

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
        self.safe_distance = 5.0
        self.max_steer = 0.8

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
            _raw_npc_list: list,

            _tl_manager: traffic_light_manager.TrafficLights,
            _car: cars.Car,
            _destination: carla.Location
    ):
        target_speed = self.max_speed
        should_break = False

        npc_list, npc_distances = self.process_npc_list(_map=_map, _router=_router, _npc_list=_raw_npc_list)
        if len(npc_list) > 0:
            nearest_npc = npc_list[0]
            npc_distance = npc_distances[nearest_npc]

            if npc_distance < self.safe_distance:
                should_break = True
                target_speed = 0

            elif npc_distance < 2 * self.safe_distance:
                target_speed = 0

            elif npc_distance < 3 * self.safe_distance:
                npc_velocity = nearest_npc.get_velocity()
                npc_speed = 3.6 * math.sqrt(npc_velocity.x ** 2 + npc_velocity.y ** 2)
                if npc_speed < _car.speed:
                    target_speed = npc_speed

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
            _previous_control=_car.control,
            _should_stop=should_break,
            _target_speed=target_speed,
            _current_speed=_car.speed,
            _dest=_destination.location,
            _now_at=_car.location,
            _forward_v=_car.forward,
        )
        _car.inject_control(control)

    @staticmethod
    def process_npc_list(
            _map: carla.Map,
            _router: router.Router,
            _npc_list: list
    ):
        _relevant_npc_list = []
        _npc_distances = {}
        for npc in _npc_list:
            npc_location = npc.get_location()
            wp = _map.get_waypoint(npc_location)
            if (wp.road_id, wp.lane_id) in _router.road_lane_pairs:
                _relevant_npc_list.append(npc)
                _npc_distances[npc] = _router.distance_to_(
                    _router.get_i_in_path(
                        npc.get_location()
                    )
                )
        sorted(
            _relevant_npc_list,
            key=lambda k: _npc_distances[k]
        )
        return _relevant_npc_list, _npc_distances
