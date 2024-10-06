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

        # prepare for synchronization
        to_sync = []
        # iterating dict gives keys
        for i in self.settings:
            actor: carla.TrafficLight = _bridge.get_actors(ids=[i])[0]
            others = actor.get_group_traffic_lights()
            for tl in others:
                tl.set_green_time(0.1)
                tl.set_yellow_time(0.1)
                tl.set_red_time(0.1)

            group = [tl for tl in actor.get_group_traffic_lights() if tl.id != i]
            group_n = len(group)
            group_yellow_time = 2.0
            group_green_time = (self.settings[i]['red_time'] - group_yellow_time * group_n) / group_n

            self.targets.append(actor)

            to_sync.append((
                actor,
                self.settings[i],
                group,
                group_yellow_time,
                group_green_time
            ))
            self.__targets_i_in_path.append(
                _router.get_i_in_path(
                    actor_location=_bridge.map.get_waypoint(
                        actor.get_location()
                    ).transform.location
                )
            )

            while len(to_sync) > 0:
                _bridge.world.wait_for_tick()
                self.__try_sync(to_sync)

    @staticmethod
    def __try_sync(to_sync):

        for row in to_sync:
            actor, setting, others, yellow_time, green_time = row
            if not actor.is_frozen() and actor.get_state() == setting['initial_state']:
                actor.set_green_time(setting['green_time'])
                actor.set_yellow_time(setting['yellow_time'])
                # red time is defined by other's g and y in group
                actor.set_red_time(0)
                for tl in others:
                    tl.set_green_time(green_time)
                    tl.set_yellow_time(yellow_time)
                    tl.set_red_time(0)
                to_sync.remove(row)

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
