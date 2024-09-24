import carla

import bridge
import cars
import router
from router import Router
from calculation_delegate import distance_in_route


class TrafficLights(object):
    def __init__(self, _bridge: bridge.CarlaBridge, _router: router.Router, _initial_settings):
        self.settings = _initial_settings
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = []
        self.distance_to_targets = []
        self.__targets_i_in_path = []
        # iterating dict gives keys
        for i in self.settings:
            actor: carla.TrafficLight = _bridge.get_actors(ids=[i])[0]
            actor.set_state(self.settings[i]['initial_state'])
            actor.set_green_time(self.settings[i]['green_time'])
            actor.set_yellow_time(self.settings[i]['yellow_time'])
            actor.set_red_time(0)

            others = [tl for tl in actor.get_group_traffic_lights() if tl.id != i]
            others_yellow_time = self.settings[i]['yellow_time']
            others_green_time = (self.settings[i]['red_time'] -
                                 (others_yellow_time * len(others))
                                 ) / len(others)

            for tl in others:
                tl.set_green_time(others_green_time)
                tl.set_yellow_time(others_green_time)
                tl.set_red_time(0)

            self.targets.append(actor)
            self.__targets_i_in_path.append(
                _router.get_i_in_path(
                    actor_location=_bridge.map.get_waypoint(
                        actor.get_location()
                    ).transform.location
                )
            )

    def update_distances(self, _router: Router):
        params = []
        for _i, target_i in enumerate(self.__targets_i_in_path):
            params.append(
                (
                    target_i,
                    _router.distance_to_(target_i),
                    self.targets[_i]
                )
            )

        self.__targets_i_in_path = []
        self.distance_to_targets = []
        self.targets = []
        for p_i, p_d, p_t in sorted(params, key=lambda p: p[1]):
            self.__targets_i_in_path.append(p_i)
            self.distance_to_targets.append(p_d)
            self.targets.append(p_t)

