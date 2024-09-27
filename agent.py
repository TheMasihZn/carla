import carla

import cars
import traffic_light_manager
from calculation_delegate import location_equal
from controller import PIDController
import math


class Agent(object):
    # noinspection PyArgumentList
    def __init__(
            self,
            _npc_list,
            _traffic_light_manager
    ):
        self.npc_list = _npc_list
        self.traffic_lights = _traffic_light_manager
        self.target_speed = 40
        self.safe_distance = 5.0
        self.max_steer = 0.8

        self.done_once = False
        self.start_dist = None
        self.stop_dist = None
        self.max_speed = 0.0

        self.pid = PIDController(
            # max_steering=0.8
        )
    def __front_car_distance(self, next_dest: carla.Transform):

        for npc in self.npc_list:
            if location_equal(
                    next_dest.location,
                    npc.transform.location,
                    self.safe_distance
            ):
                return (
                    npc.transform.location.distance_to(
                        next_dest.location
                    )
                )
        return False

    def on_tick(self, _car: cars.Car, _tl_manager: traffic_light_manager.TrafficLights, _destination: carla.Location):

        d_to_tl = _tl_manager.distance_to_targets[0]
        d = 0.0
        control = _car.control
        if self.max_speed < _car.speed:
            self.max_speed = _car.speed
        if not self.done_once:

            if _car.speed / 3.6 <= 10.0:
                control.throttle = 1.0
                control.brake=0.0
            else:
                if not self.start_dist:
                    self.start_dist = _car.location
                control.throttle = 0.0
                control.brake=0.0
                self.done_once = True
        else:
            if _car.speed == 0.0:
                if not self.stop_dist:
                    self.stop_dist = _car.location
                    d = math.sqrt((self.start_dist.x - self.stop_dist.x)**2 + (self.start_dist.y - self.stop_dist.y)**2)

                    # should_stop = False
        # if d_to_tl < self.safe_distance:
        #     control.throttle = 0
        #     control.brake = 1
        #     return control
        #
        # elif d_to_tl < self.safe_distance * 4:
        #     should_stop = True
        #
        # control = self.pid.get_new_control(target_speed=40.0, destination=_destination)
        _car.inject_control(control)


