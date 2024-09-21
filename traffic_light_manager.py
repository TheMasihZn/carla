import carla
import bridge


class TrafficLightManager(object):
    def __init__(self, _bridge, _initial_settings):
        self.settings = _initial_settings
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = _bridge.get_actors(ids=list(self.settings.keys()))
        for tl in self.targets:
            tl.set_state(self.settings[tl.id]['initial_state'])
            tl.set_green_time(self.settings[tl.id]['green_time'])
            tl.set_yellow_time(self.settings[tl.id]['yellow_time'])
            tl.set_red_time(self.settings[tl.id]['red_time'])
