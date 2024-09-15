import carla
import csv
import matplotlib.pyplot as plt

import bridge
from calculation_delegate import location_equal


class Router(object):
    def __init__(self, _bridge: bridge.CarlaBridge, spawn_hints: bool):
        self.path = self.__read_path_from_file(_bridge)
        self.path_taken = []
        self.route = self.__generate_route(_bridge, self.path, spawn_hints)
        self.traffic_lights = _bridge.traffic_lights

    def on_tick(self, _vehicle_transform: carla.Transform):
        if _vehicle_transform not in self.path_taken:
            self.path_taken.append(_vehicle_transform)
        for step in self.route:
            if location_equal(
                    step['transform'].location,
                    _vehicle_transform.location,
                    1.5
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

    def next_n(self, n=1):
        if not self.route:
            return None
        return self.route[0:n]

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
    def __generate_route(_bridge, _path, spawn_hints):
        _route = []
        hint_bp = _bridge.blueprint_library.filter(
            'static.prop.ironplank'
        )[0]
        for transform in _path:
            hint = None if not spawn_hints else _bridge.spawn_actor(hint_bp, transform)
            _route.append(
                {
                    'transform': transform,
                    'hint': hint
                }
            )
        return _route

    def draw_path(self, waypoints, start_transform, traffic_lights=None):
        plt.figure(figsize=(7, 7))
        x_vals = [waypoint.transform.location.x for waypoint in waypoints]
        y_vals = [waypoint.transform.location.y for waypoint in waypoints]
        plt.scatter(x_vals, y_vals, s=1, color='black')

        if self.path:
            x_path = [path.location.x for path in self.path]
            y_path = [path.location.y for path in self.path]
            # path
            plt.scatter(
                x_path,
                y_path,
                s=1,
                color='yellow'
            )
            # start
        if start_transform:
            plt.scatter(
                start_transform.location.x,
                start_transform.location.y,
                s=15,
                color='blue'
            )
        if traffic_lights:
            x_all_traffic_lights = [tl.get_location().x for tl in traffic_lights.all]
            y_all_traffic_lights = [tl.get_location().y for tl in traffic_lights.all]
            plt.scatter(x_all_traffic_lights, y_all_traffic_lights, s=30, color='white')

            x_chosen_traffic_lights = [tl.get_location().x for tl in traffic_lights.target]
            y_chosen_traffic_lights = [tl.get_location().y for tl in traffic_lights.target]
            plt.scatter(x_chosen_traffic_lights, y_chosen_traffic_lights, s=30, color='green')

        # Plot the road network
        plt.title("2D Road-Only Map")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()
