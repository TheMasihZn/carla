import carla
import bridge
from calculation_delegate import equal
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
    def __front_car_distance(self, next_dest):

        for npc in self.npc_list:
            if equal(
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

    def __traffic_light_distance(self, current_waypoint, vehicle):
        # in_vicino = [tl for tl in self.traffic_lights.all if
        #              equal(tl.get_transform().location, vehicle.get_transform().location, 3 * self.safe_distance)]
        # for tl in in_vicino:
        #     if tl.get_state() == carla.TrafficLightState.Red:
        #         if current_waypoint in tl.get_stop_waypoints():
        #             if (tl.get_red_time() - tl.get_elapsed_time()) > (self.safe_distance / self.target_speed):
        #                 return self.safe_distance

        return False

    def should_stop(self, current_waypoint, player: carla.Vehicle, next_dest):

        predict_transform = next_dest
        # v = player.get_velocity()
        # if v.length() > 0:
        #     next_transform += carla.Location(
        #         v.make_unit_vector().dot(
        #             carla.Vector3D(x=self.safe_distance,
        #                            y=self.safe_distance,
        #                            z=0.0)
        #         )
        #     )
        front_car_distance = self.__front_car_distance(next_dest)

        traffic_light_distance = self.__traffic_light_distance(current_waypoint, player)

        stop_distance = 0
        if front_car_distance:
            stop_distance += front_car_distance
        if traffic_light_distance:
            stop_distance += traffic_light_distance

        return stop_distance if stop_distance > 0 else False
