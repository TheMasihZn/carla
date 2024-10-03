import carla
import csv
import matplotlib.pyplot as plt

from bridge import CarlaBridge
from calculation_delegate import location_equal, distance_in_route


class Router(object):
    def __init__(self, _bridge: CarlaBridge, route_file_path: str, spawn_hints: bool, route_z=0.1):
        self.spawn_hints = spawn_hints
        self.path = self.__read_path_from_file(route_file_path, route_z)
        self.last_transform = self.path[0]
        self.route = []
        self.__current_index_in_route = 0
        self.__route_header = 0
        self.__hint_header = 0
        self.update_cache_route()

    def on_tick(self, _current_transform: carla.Transform, _speed: float, _bridge: CarlaBridge):
        if _current_transform != self.last_transform:
            self.last_transform = _current_transform
        for step in self.route:
            if location_equal(
                    step.location,
                    _current_transform.location,
                    1.5
            ):
                self.route.remove(step)
                self.__current_index_in_route += 1
                self.__current_index_in_route %= len(self.path)
            else:
                break
        self.update_cache_route()
        if self.spawn_hints:
            self.draw_hints(_speed, _bridge)

    def next_destination(self):
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
        if len(self.route) > n_batch / 2:
            return
        for transform in self.path[self.__route_header:self.__route_header + n_batch]:
            self.route.append(transform)
        self.__route_header += n_batch
        if self.__route_header >= len(self.path):
            self.__route_header -= len(self.path)
            self.route += self.path[0:self.__route_header]

    def distance_to_(self, i_in_path) -> float:
        return distance_in_route(self.__current_index_in_route, i_in_path, self.path)

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

    def draw_hints(
            self, _speed, _bridge,
            _color=carla.Color(r=0, g=125, b=125, a=125),
            n=20,
            resolution=2
    ):
        _header_after = self.__hint_header + resolution

        # to avoid over draw
        if _header_after > self.__current_index_in_route + n:
            return

        _path = self.path[self.__hint_header:_header_after]
        if _header_after >= len(self.path):
            _header_after -= len(self.path)
            _path += self.path[:_header_after]

        for i in range(1, len(_path)):
            # division by n<1 = infinite time
            # hint_distance = distance_in_route(self.__current_index_in_route, self.__hint_header, self.path)
            # time_step = (_speed + 0.01) / resolution
            # if _speed < 1:
            #     _time = i * 0.5
            # elif _speed < (hint_distance / resolution):
            #     _time = i * time_step * 0.5
            # else:
            #     _time = hint_distance / (_speed + 0.01)

            _time = i * 0.5
            _bridge.world.debug.draw_line(
                begin=_path[i - 1].location,
                end=_path[i].location,
                thickness=0.08,
                color=_color,
                life_time=_time,
            )
            self.__hint_header += 1
            if self.__hint_header > len(self.path):
                self.__hint_header -= len(self.path)

    def get_i_in_path(self, actor_location: carla.Location) -> int:
        for i, step in enumerate(self.path):
            if location_equal(step.location, actor_location, 5):
                return i

        raise Exception(
            "actor location not in path"
            + "\nif you're sure it is, it's because of unreliable id assignment"
            + "\n(restart simulator server)"
        )
