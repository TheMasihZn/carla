import carla
import csv
import matplotlib.pyplot as plt

from bridge import CarlaBridge
from calculation_delegate import location_equal


class Router(object):
    def __init__(self, _bridge: CarlaBridge, route_file_path: str, spawn_hints: bool, route_z=0.1):
        self.spawn_hints = spawn_hints
        self.path = self.__read_path_from_file(route_file_path, route_z)
        self.last_transform = self.path[0]
        self.route = []
        self.__header = 0
        self.__last_hint_index = 0
        self.update_cache_route()

    def on_tick(self, _current_transform: carla.Transform):
        if _current_transform != self.last_transform:
            self.last_transform = _current_transform
        for step in self.route:
            if location_equal(
                    step.location,
                    _current_transform.location,
                    1.5
            ):
                self.route.remove(step)
            else:
                break
        self.update_cache_route()

    def next_destination(self):
        if not self.route:
            return None
        return self.route[0]

    # noinspection PyTypeChecker
    @staticmethod
    def __read_path_from_file(route_file_path: str, route_z=0.1) -> list:
        _route = []
        for line in csv.DictReader(open(route_file_path, 'r')):
            rotation = carla.Rotation(float(line['pitch']), float(line['yaw']), float(line['roll']))
            location = carla.Location(float(line['x']), float(line['y']), route_z)
            transform = carla.Transform(location, rotation)
            _route.append(transform)
        return _route

    def update_cache_route(self, n_batch=50):
        if len(self.route) > n_batch:
            return
        for transform in self.path[self.__header:self.__header + (len(self.route) - n_batch)]:
            self.route.append(transform)
        self.__header += len(self.route) - n_batch

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

    # def distance_in_route(self, l1: carla.Location, l2: carla.Location) -> float:
    #     d = 0
    #     for transform in self.path:
    #
    #
    #     return d

    def draw_hints(self, _car, _bridge,
                   _color=carla.Color(r=0, g=125, b=125, a=125)
                   ):
        _path = self[self.__last_hint_index:]

        for i in range(1, len(_path) - 1):
            _bridge.world.debug.draw_line(
                begin=_path[i].location,
                end=_path[i + 1].location,
                thickness=0.1,
                color=_color,
                life_time=_bridge.settings.max_substep_delta_time
            )
        # at last draw arrow
        _bridge.world.debug.draw_arrow(
            begin=_path[-2].location,
            end=_path[-1].location,
            arrow_size=1,
            color=_color,
            life_time=_bridge.settings.max_substep_delta_time
        )
        _bridge.last_hint = _path[-2]
