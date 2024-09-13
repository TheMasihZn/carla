import carla
import csv
import matplotlib.pyplot as plt

import bridge
from calculation_delegate import equal


class Router(object):
    def __init__(self, _bridge: bridge.CarlaBridge):
        self.path = self.__read_path_from_file(_bridge)
        self.path_taken = []
        self.route = self.__generate_route(_bridge, self.path)
        self.traffic_lights = _bridge.traffic_lights
        self.target_lights = _bridge.target_lights

    def on_tick(self, _vehicle_transform: carla.Transform):
        if _vehicle_transform not in self.path_taken:
            self.path_taken.append(_vehicle_transform)
        for step in self.route:
            if equal(
                    step['transform'].location,
                    _vehicle_transform.location,
                    3
            ):
                self.route.remove(step)
                if step['hint']:
                    step['hint'].destroy()
            else:
                break

    def next_destination(self):
        if not self.route:
            return None
        return self.route[0]

    def destroy(self):
        for route in self.route:
            if route['hint']:
                route['hint'].destroy()
                route['hint'] = None

    # noinspection PyTypeChecker
    @staticmethod
    def __read_path_from_file(_bridge: bridge.CarlaBridge):
        _route = []
        for line in csv.DictReader(open('route.csv', 'r')):
            rotation = carla.Rotation(float(line['pitch']), float(line['yaw']), float(line['roll']))
            location = carla.Location(float(line['x']), float(line['y']), float(line['z']))
            transform = carla.Transform(location, rotation)
            _route.append(transform)
        return _route

    @staticmethod
    def __generate_route(_bridge, _path):
        _route = []
        hint_bp = _bridge.blueprint_library.filter(
            'static.prop.ironplank'
        )[0]
        for transform in _path:
            # hint = _bridge.world.spawn_actor(hint_bp, transform)
            hint = None
            _route.append(
                {
                    'transform': transform,
                    'hint': hint
                }
            )
        return _route

    def draw_route(self, waypoints):
        plt.figure(figsize=(7, 7))
        x_vals = [waypoint.transform.location.x for waypoint in waypoints]
        y_vals = [waypoint.transform.location.y for waypoint in waypoints]
        plt.scatter(x_vals, y_vals, s=1, color='black')

        if self.route:
            x_route = [route['transform'].location.x for route in self.route]
            y_route = [route['transform'].location.y for route in self.route]
            # path
            plt.scatter(
                x_route,
                y_route,
                s=1,
                color='yellow'
            )
            # start
            plt.scatter(
                self.route[0]['transform'].location.x,
                self.route[0]['transform'].location.y,
                s=15,
                color='blue'
            )

        x_all_traffic_lights = [tl.get_location().x for tl in self.traffic_lights]
        y_all_traffic_lights = [tl.get_location().y for tl in self.traffic_lights]
        plt.scatter(x_all_traffic_lights, y_all_traffic_lights, s=30, color='white')

        x_chosen_traffic_lights = [tl.get_location().x for tl in self.target_lights]
        y_chosen_traffic_lights = [tl.get_location().y for tl in self.target_lights]
        plt.scatter(x_chosen_traffic_lights, y_chosen_traffic_lights, s=30, color='green')

        # Plot the road network
        plt.title("2D Road-Only Map")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()
