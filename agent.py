import carla
import bridge
from controller import VehiclePIDController
from router import Router
from calculation_delegate import equal
from carla import TrafficLightState as LightState


class Agent(object):
    def __init__(
            self,
            vehicle: carla.Vehicle,
            _bridge: bridge.CarlaBridge
    ):
        self.npc_list = _bridge.npc_list
        self.target_speed = 40
        self.safe_distance = 5.0  # meters

    def stop_in(self, distance: float = 0.0) -> carla.VehicleControl:
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False
        control.manual_gear_shift = False
        return control

    # detect if there is a hazard
    def detect_hazard(self, predict_transform):
        for npc in self.npc_list:
            if equal(
                    predict_transform.location,
                    npc.transform.location,
                    self.safe_distance
            ):
                return (
                    npc.transform.location.distance_to(
                        npc.transform.location
                    )
                )
        return False

    def traffic_light_stop(self, vehicle):
        if vehicle.is_at_traffic_light():
            if vehicle.get_traffic_light_state() == LightState.Red:
                return self.safe_distance
            return False
