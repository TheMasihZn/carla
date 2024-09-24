import carla

import traffic_light_manager
from calculation_delegate import location_equal


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

    # def stop_in(self, distance: float = 0.0):
    #     self.control.throttle = 0.0
    #     self.control.brake = 1.0
    #     self.control.hand_brake = (self.safe_distance > distance)

    # detect if there is a hazard
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

    def __traffic_light_distance(
            self,
            _tl_manager: traffic_light_manager.TrafficLights,
            reference_waypoint):
        d = min(_tl_manager.distance_to_targets)
        if d < self.safe_distance * 4:
            return d

        return False

    def should_stop(self, next_n_waypoints):
        """
        :param next_n_waypoints: how many future waypoints to analyse
        """
        for step_waypoint in next_n_waypoints:
            stop_distance = 0
            front_car_distance = self.__front_car_distance(step_waypoint.transform)
            if front_car_distance:
                stop_distance += front_car_distance

            traffic_light_distance = self.__traffic_light_distance(step_waypoint)
            if traffic_light_distance:
                stop_distance += traffic_light_distance

            if stop_distance > 0:
                return stop_distance

        return False
