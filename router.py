import carla
import csv
import matplotlib.pyplot as plt

import car_manager
from bridge import CarlaBridge
from calculation_delegate import location_equal, distance_in_route


class Router(object):
    def __init__(self, _bridge: CarlaBridge, route_file_path: str, spawn_hints: bool, route_z=0.1):
        self.spawn_hints = spawn_hints
        self.path = self.__read_path_from_file(_bridge, route_file_path, route_z)
        self.last_transform = self.path[0]
        self.route = []
        self.__current_index_in_route = 0
        self.__route_header = 0
        self.__hint_header = 0
        self.update_cache_route()

    def on_tick(
            self,
            _car_manager: car_manager.CarManager,
            _bridge: CarlaBridge
    ):
        if _car_manager.ego.transform != self.last_transform:
            self.last_transform = _car_manager.ego.transform
        for i, step in enumerate(self.route):
            for car in [_car_manager.ego, *_car_manager.npc_list]:
                if location_equal(
                        step.location,
                        car.location,
                        1.5
                ):
                    car.i_on_path = i
                    if isinstance(car, car_manager.Ego):
                        self.route.remove(step)
                        car.i_on_path %= len(self.path)
                        self.__current_index_in_route = car.i_on_path
                    elif isinstance(car, car_manager.NPC):
                        car.distance_ego_to_car = self.distance_to_(car.i_on_path)

        self.update_cache_route()
        if self.spawn_hints:
            self.draw_hints(_bridge)

    def next_destination(self):
        return self.route[0]

    # noinspection PyTypeChecker
    @staticmethod
    def __read_path_from_file(_bridge, route_file_path: str, route_z=0.1) -> list:
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

    # noinspection PyArgumentList
    def draw_hints(
            self, _bridge,
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
            _time = i * 0.8
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

    def get_i_in_path(self, actor_location: carla.Location, threshold=5) -> int:
        for i, step in enumerate(self.path):
            if location_equal(step.location, actor_location, threshold):
                return i

        raise Exception(
            "actor location not in path"
            + "\nif you're sure it is, it's because of unreliable id assignment"
            + "\n(restart simulator server)"
        )
