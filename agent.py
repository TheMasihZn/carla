import carla

import bridge
import car_manager
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
        self.n_projections = 10
        self.max_speed = 40
        self.max_steer = 0.8
        self.safe_distance = 5.0

        self.target_speed = self.max_speed

        self.traffic_lights: traffic_light_manager.TrafficLights = _traffic_light_manager

        # self.done_once = False
        # self.start_dist = None
        # self.stop_dist = None
        # self.max_speed = 0.0

        self.pid = PIDController()

    def on_tick(
            self,
            _car_manager: car_manager.CarManager,
            _tl_manager: traffic_light_manager.TrafficLights,
            _router: router.Router,
            _dt: float
    ):
        controls = [_car_manager.ego.control]
        next_destinations = _router.next_(10)

        for projection_step in range(len(next_destinations)):
            should_break = False

            relevant_npc_list = [
                (
                    (
                            n.distance_ego_to_car +
                            n.speed_mps * (projection_step * _dt)
                    ),
                    n.speed
                )
                for n in _car_manager.npc_list if n.distance_ego_to_car > 0]
            sorted(
                relevant_npc_list,
                key=lambda _distance_speed: _distance_speed[0]
            )
            if len(relevant_npc_list) > 0:
                d, s = relevant_npc_list[0]
                if d < 3 * self.safe_distance:
                    should_break = True

                elif d < 4 * self.safe_distance:
                    self.target_speed = s

                elif d > 8 * self.safe_distance:
                    self.target_speed = self.max_speed

            next_light = self.traffic_lights.targets[0]
            projected_d_to_ego = next_light.distance_from_ego + (
                    (_dt * projection_step)
                    *
                    _car_manager.ego.speed_mps
                    *
                    (-1 if next_light.distance_from_ego > 0 else 1)  # todo fix d never < 0
            )
            if next_light.state == carla.TrafficLightState.Red:
                if location_equal(_car_manager.ego.location, next_light.location,
                                  7 * self.safe_distance):
                    should_break = True

            control = self.pid.get_control_for_t(
                _previous_control=_car_manager.ego.control,
                _should_stop=should_break,
                _target_speed=self.target_speed,
                _current_speed=_car_manager.ego.speed,
                _dest=_router.next_destination().location,
                _now_at=_car_manager.ego.location,
                _forward_v=_car_manager.ego.forward,
                _dt=_dt * projection_step
            )

            controls.append(control)
        _car_manager.ego.inject_control(controls[0])

