import carla
import bridge


class TrafficLights(object):
    def __init__(self, _bridge, _initial_settings):
        self.settings = _initial_settings
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = []
        # iterating dict gives keys
        for i in self.settings:
            actor = _bridge.get_actors(ids=[i])[0]
            actor.set_state(self.settings[i]['initial_state'])
            actor.set_green_time(self.settings[i]['green_time'])
            actor.set_yellow_time(self.settings[i]['yellow_time'])
            actor.set_red_time(self.settings[i]['red_time'])
            self.targets.append(actor)
