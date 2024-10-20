import mpc_agent
import car_manager
import cars
import controller
import fuel_cost
import router
import traffic_light_manager
from copy import deepcopy


class MPC:
    def __init__(self):
        self.n_projection_steps = 10

    # def step(
    #         self,
    #         _traffic_lights: traffic_light_manager.TrafficLights,
    #         _car_manager: car_manager.CarManager
    # ):
    #     for i in range(self.n_projection_steps):
    #         self.ego.speed
    #
    #
    #         for npc in [npc for npc in _car_manager.npc_list if npc.i_on_path > 0]:
    #             pass

    def next_step(
            self,
            _pid: controller.PIDController,
            # _agent: mpc_agent.MPCAgent,
            _router: router.Router,
            _traffic_lights: traffic_light_manager.TrafficLights,
            _car_manager: car_manager.CarManager
    ):
        self.n_projection_steps = self.n_projection_steps
        should_break = False
        # target_speed = _agent.target_speed
        current_speed = _car_manager.ego.speed
        next_destinations = _router.next_(10)
        current_location = _car_manager.ego.location
        forward_vector = _car_manager.ego.forward

        controls = [_car_manager.ego.control]

        for step in range(next_destinations):

            control = _pid.get_new_control(
                _previous_control=controls[-1],
                _should_stop=should_break,
                # _target_speed=target_speed,
                _current_speed=current_speed,
                _dest=next_destinations,
                _now_at=current_location,
                _forward_v=forward_vector,
            )
            controls.append(control)
        return control

