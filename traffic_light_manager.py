import carla

import bridge
import cars
import router
from router import Router
from calculation_delegate import distance_in_route


class Light:
    def __init__(
            self,
            _actor: carla.TrafficLight,
            _i_in_path: int,
            _initial_state: carla.TrafficLightState,
            _green_time: float,
            _yellow_time: float,
            _red_time: float,
    ):
        self.actor: carla.TrafficLight = _actor
        self.initial_state: carla.TrafficLightState = _initial_state
        self.i_in_path: int = _i_in_path
        self.green_time: float = _green_time
        self.yellow_time: float = _yellow_time
        self.red_time: float = _red_time
        self.distance_from_ego: float = -1.
        self.time_to_next_green: float = -1.
        self.state: carla.TrafficLightState = carla.TrafficLightState.Unknown

        self.__last_elapsed_time = 0.0
        self.is_synced = False
        self.sync_group()

        self.__group = self.actor.get_group_traffic_lights()
        self.__others = [*self.__group]
        self.__others.remove(self.actor)
        for tl in self.__group:
            tl.set_green_time(0.05)
            tl.set_yellow_time(0.05)
            tl.set_red_time(0.05)

        others_n = len(self.__others)
        self.__others_yellow_time = 2.0
        self.__others_green_time = (self.red_time - self.__others_yellow_time * others_n) / others_n

    def on_tick(
            self,
            _router: router.Router,
    ):
        new_state = self.actor.get_state()
        if new_state != self.state:
            self.__last_elapsed_time = 0.0
            if new_state == carla.TrafficLightState.Green:
                self.time_to_next_green = - self.green_time
            self.state = new_state
        current_elapsed_time = self.actor.get_elapsed_time()
        self.time_to_next_green += current_elapsed_time - self.__last_elapsed_time
        self.__last_elapsed_time = current_elapsed_time
        self.distance_from_ego = _router.distance_to_(self.i_in_path)

    def sync_group(self):
        if not self.actor.is_frozen() and self.actor.get_state() == self.initial_state:
            self.actor.set_green_time(self.green_time)
            self.actor.set_yellow_time(self.yellow_time)
            # red time is defined by other's g and y in group
            self.actor.set_red_time(0)
            for tl in self.__others:
                tl.set_green_time(self.__others_green_time)
                tl.set_yellow_time(self.__others_yellow_time)
                tl.set_red_time(0)

            self.is_synced = True


class TrafficLights(object):
    def __init__(self, _bridge: bridge.CarlaBridge, _router: router.Router, _initial_settings):

        self.settings = _initial_settings
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = []
        self.targets_time_elapsed_to_green = []
        self.__targets_i_in_path = []

        # iterating dict gives keys
        for i in self.settings:
            actor: carla.TrafficLight = _bridge.get_actors(ids=[i])[0]
            i_in_path = _router.get_i_in_path(
                actor_location=_bridge.map.get_waypoint(actor.get_location()).transform.location)

            self.targets.append(
                Light(
                    actor,
                    i_in_path,
                    self.settings[i]['initial_state'],
                    self.settings[i]['green_time'],
                    self.settings[i]['yellow_time'],
                    self.settings[i]['red_time']
                )
            )

        for tl in self.targets:
            tl: Light = tl
            if not tl.is_synced:
                while not tl.is_synced:
                    _bridge.world.wait_for_tick()
                    tl.sync_group()

    def on_tick(self, _router: Router):
        for tl in self.targets:
            tl.on_tick(_router)
        sorted(self.targets, key=lambda l: l.distance_from_ego)
        params = []
        for _i, target_i in enumerate(self.__targets_i_in_path):
            params.append(
                (
                    target_i,
                    _router.distance_to_(target_i),
                    self.targets[_i],
                    self.targets_time_elapsed_to_green[_i],
                    self.__targets_total_cycle_time[_i]
                )
            )

        for p_i_in_path, p_distance, p_light, p_green_time, g_total_time in sorted(params, key=lambda p: p[1]):
            self.__targets_i_in_path.append(p_i_in_path)
            self.distance_to_targets.append(p_distance)
            self.targets.append(p_light)
            self.__targets_total_cycle_time.append(g_total_time)
            self.targets_time_elapsed_to_green.append((p_green_time + self.__step_delta_t) % g_total_time)
