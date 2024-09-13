import carla
import bridge


class TrafficLightManager(object):
    def __init__(self, _bridge):
        self.settings = {
            11: {
                'initial_state': carla.TrafficLightState.Red,
                'green_time': 10.0,
                'yellow_time': 5.0,
                'red_time': 25.0,
            },
            13: {
                'initial_state': carla.TrafficLightState.Green,
                'green_time': 15.0,
                'yellow_time': 5.0,
                'red_time': 20.0,
            },
            20: {
                'initial_state': carla.TrafficLightState.Red,
                'green_time': 12.0,
                'yellow_time': 4.0,
                'red_time': 24.0,
            },
        }
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = self.__get_target_lights(_bridge)

    def __get_target_lights(self, _bridge):
        _target_lights = _bridge.get_actors(ids=list(self.settings.keys()))
        for tl in _target_lights:
            tl.set_state(self.settings[tl.id]['initial_state'])
            tl.set_green_time(self.settings[tl.id]['green_time'])
            tl.set_yellow_time(self.settings[tl.id]['yellow_time'])
            tl.set_red_time(self.settings[tl.id]['red_time'])
        return _target_lights
