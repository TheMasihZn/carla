import carla
import bridge
from calculation_delegate import (location_equal, rotation_equal, transform_equal)
from carla import TrafficLightState as LightState


class Agent(object):
    # noinspection PyArgumentList
    def __init__(
            self,
            vehicle: carla.Vehicle,
            _bridge: bridge.CarlaBridge
    ):
        self.npc_list = _bridge.npc_list
        self.traffic_lights = _bridge.traffic_lights
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
                        next_dest.transform.location
                    )
                )
        return False

    def __traffic_light_distance(self, reference_waypoint):
        in_vicinity = [
            tl for tl in self.traffic_lights.all
            if location_equal(
                tl.get_transform().location,
                reference_waypoint.transform.location,
                location_threshold=3 * self.safe_distance
            )
        ]
        for tl in in_vicinity:
            if reference_waypoint.lane_id() in [wp.lane_id for wp in tl.get_affected_lane_waypoints()]:
                if tl.get_state() == carla.TrafficLightState.Red:
                    if (tl.get_red_time() - tl.get_elapsed_time()) > (self.safe_distance / self.target_speed):
                        return self.safe_distance

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
