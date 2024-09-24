import carla
import bridge


class TrafficLights(object):
    def __init__(self, _bridge, _initial_settings):
        self.settings = _initial_settings
        self.all = _bridge.get_actors(filter_key='traffic.traffic_light')
        self.targets = []
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
