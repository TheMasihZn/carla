import random
import carla
import calculation_delegate as helper

random.seed(0)
SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
DestroyActor = carla.command.DestroyActor


class CarlaBridge(object):

    # noinspection PyUnresolvedReferences
    def __init__(self):
        self.client = carla.Client('127.0.0.1', 2000)
        self.world = self.client.get_world()

        self.settings = self.world.get_settings()
        self.map: carla.Map = self.world.get_map()

        self.traffic_manager = self.client.get_trafficmanager(8000)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        # self.traffic_manager.global_percentage_speed_difference(70.0)
        self.traffic_manager.set_respawn_dormant_vehicles(True)
        self.traffic_manager.set_random_device_seed(0)

        self.spectator = self.world.get_spectator()

        self.spawn_points = list(self.map.get_spawn_points())
        random.shuffle(self.spawn_points)

        self.blueprint_library = self.world.get_blueprint_library()
        _blueprints = self.blueprint_library.filter('vehicle.*')
        self.blueprints = [x for x in _blueprints if x.get_attribute('base_type') == 'car']

        self.cars = []
        self.npc_list = []
        self.hints = []

        self.traffic_lights = list(self.world.get_actors().filter('traffic.traffic_light'))

        def get_target_traffic_light_settings(settings: dict):
            _target_lights = self.world.get_actors(list(settings.keys()))
            for tl in _target_lights:
                tl.set_state(settings[tl.id]['initial_state'])
                tl.set_green_time(settings[tl.id]['green_time'])
                tl.set_yellow_time(settings[tl.id]['yellow_time'])
                tl.set_red_time(settings[tl.id]['red_time'])
            return _target_lights

        self.target_lights = get_target_traffic_light_settings(
            settings={
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
        )

    def reset_actors(self):
        while len(self.cars) > 0:
            car = self.cars[0]
            for response in self.client.apply_batch_sync(
                    [DestroyActor(car)],
                    True
            ):
                if response.error:
                    print(response.error)
                else:
                    self.cars.remove(car)

    def go_async(self):
        settings = self.settings
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(False)
        print('async')

    def go_sync(self):
        settings = self.settings
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.5
        self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(True)
        print('sync')

    def spawn_teraffic(self, n_cars):
        spawn_batch = []
        while n_cars != 0:
            blueprint = random.choice(self.blueprints)
            blueprint.set_attribute('role_name', 'autopilot')
            point = random.choice(self.spawn_points)
            cmd = SpawnActor(blueprint, point).then(
                SetAutopilot(
                    FutureActor,
                    True,
                    self.traffic_manager.get_port()
                )
            )
            for response in self.client.apply_batch_sync([cmd], True):
                if not response.error:
                    n_cars -= 1
                    self.cars.append(self.world.get_actors().find(response.actor_id))

    # def draw(self):
    #     helper.draw_route(
    #         waypoints=self.map.generate_waypoints(distance=2.0),
    #         route=self.route,
    #         all_traffic_lights=self.traffic_lights,
    #         chosen_traffic_lights=self.target_lights
    #     )

    # noinspection PyArgumentList
    def generate_destinations(self, start_location):
        dest = [
            self.map.get_waypoint(carla.Location(x=109.564789, y=44.198593, z=0.000000)).transform.location,
            self.map.get_waypoint(carla.Location(x=109.946991, y=-13.351235, z=0.000000)).transform.location,
            self.map.get_waypoint(carla.Location(x=34.043774, y=-67.910881, z=0.000000)).transform.location,
            self.map.get_waypoint(carla.Location(x=-21.908228, y=-68.223061, z=0.000000)).transform.location,
            self.map.get_waypoint(carla.Location(x=-62.118881, y=-68.641869, z=0.000000)).transform.location,
            self.map.get_waypoint(carla.Location(x=-114.413689, y=59.960121, z=0.000000)).transform.location,
            start_location
        ]
        return dest

    def __delete__(self, instance):
        self.reset_actors()
        # self.destroy_hints()

        # spawn_teaffic(n_cars=20)

        # draw_route(route)
