import random
import carla
from traffic_light_manager import TrafficLightManager

random.seed(0)
SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
DestroyActor = carla.command.DestroyActor


class CarlaBridge(object):

    # noinspection PyUnresolvedReferences
    def __init__(self):
        self.artificial_actors = []
        self.client = carla.Client('127.0.0.1', 2000)
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.map: carla.Map = self.world.get_map()

        traffic_manager = self.client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_respawn_dormant_vehicles(True)
        traffic_manager.set_random_device_seed(0)
        self.traffic_manager = traffic_manager

        self.spectator = self.world.get_spectator()

        self.spawn_points = list(self.map.get_spawn_points())
        random.shuffle(self.spawn_points)

        self.blueprint_library = self.world.get_blueprint_library()
        _blueprints = self.blueprint_library.filter('vehicle.*')
        self.vehicle_blueprints = [x for x in _blueprints if x.get_attribute('base_type') == 'car']

        self.traffic_lights = TrafficLightManager(_bridge=self)

        self.cars = []
        self.npc_list = []
        self.hints = []

    def spawn_actor(self,
                    bp: carla.ActorBlueprint,
                    point: carla.Transform,
                    attach_to=None,
                    attachment_type=carla.AttachmentType.Rigid
                    ):
        actor = self.world.spawn_actor(bp, point, attach_to, attachment_type)
        self.artificial_actors.append(actor)
        return actor

    def get_actors(self, ids=None, filter_key=None):
        if ids:
            actors = self.world.get_actors(ids)
        else:
            actors = self.world.get_actors()
        if filter_key:
            actors = actors.filter(filter_key)
        return list(actors)

    def delete_artificial_actors(self):
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
            blueprint = random.choice(self.vehicle_blueprints)
            blueprint.set_attribute('role_name', 'npc')
            self.spawn_actor()
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
