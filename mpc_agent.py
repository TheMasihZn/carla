import carla

import bridge
import car_manager
import cars
import mpc_projection
import router
import traffic_light_manager
from calculation_delegate import location_equal
from controller import PIDController
import math


class MPCAgent(object):
    # noinspection PyArgumentList
    def __init__(
            self,
            _traffic_light_manager
    ):
        self.n_projections = 10
        self.tick_delta_t = 0.01
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
            _car_manager: car_manager.CarManager,
            _tl_manager: traffic_light_manager.TrafficLights,
            _router: router.Router,
            _debug_bridge: bridge.CarlaBridge = None
    ):
        controls = [_car_manager.ego.control]
        next_destinations = _router.next_(10)

        for projection_step in range(len(next_destinations)):
            should_break = False

            relevant_npc_list = [
                (
                    (
                            n.distance_ego_to_car +
                            n.speed_mps * (projection_step * self.tick_delta_t)
                    ),
                    n.speed
                )
                for n in _car_manager.npc_list if n.distance_ego_to_car > 0]
            sorted(
                relevant_npc_list,
                key=lambda _distance_speed: _distance_speed[0]
            )
            for _distance_speed in relevant_npc_list:
                d, s = _distance_speed
                if d < self.safe_distance:
                    should_break = True

                elif d < 2 * self.safe_distance:
                    self.target_speed = s

                elif d > 6 * self.safe_distance:
                    self.target_speed = self.max_speed

            next_light = self.traffic_lights.targets[0]
            if next_light.state == carla.TrafficLightState.Red:
                if location_equal(_car_manager.ego.location, next_light.location,
                                  7 * self.safe_distance):
                    should_break = True

            control = self.pid.get_new_control(
                _previous_control=_car_manager.ego.control,
                _should_stop=should_break,
                _target_speed=self.target_speed,
                _current_speed=_car_manager.ego.speed,
                _dest=_router.next_destination().location,
                _now_at=_car_manager.ego.location,
                _forward_v=_car_manager.ego.forward,
            )
            controls.append(control)
        #     todo choose a good control
        _car_manager.ego.inject_control(controls[0])
