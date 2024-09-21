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
        self.client = carla.Client('127.0.0.1', 2000)
        self.world: carla.World = self.client.get_world()
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
        self.vehicle_blueprints = [bp for bp in _blueprints if 'vehicle' in bp.tags]

        self.npc_list = []
        self.hints = []

    def spawn_actor(self,
                    bp: carla.ActorBlueprint,
                    point: carla.Transform,
                    attach_to=None,
                    attachment_type=carla.AttachmentType.Rigid,
                    destroy_at_the_end=True
                    ):
        if destroy_at_the_end:
            name = bp.get_attribute('role_name').as_str() + '__destroy'
            bp.set_attribute('role_name', name)
        # point.location.z = 4.0
        return self.world.spawn_actor(bp, point, attach_to, attachment_type)

    def get_actors(self, ids=None, filter_key=None):
        if ids:
            actors = self.world.get_actors(ids)
        else:
            actors = self.world.get_actors()
        if filter_key:
            actors = actors.filter(filter_key)
        return list(actors)

    def delete_created_actors(self):
        while True:
            destroy_list = [a for a in self.get_actors()
                            if 'role_name' in a.attributes.keys()
                            and
                            'destroy' in a.attributes['role_name']
                            ]
            if len(destroy_list) == 0:
                break
            for actor in destroy_list:
                if actor.is_alive:
                    actor.destroy()

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
        while len(self.npc_list) < n_cars:
            try:
                blueprint = random.choice(self.vehicle_blueprints)
                blueprint.set_attribute('role_name', 'npc')
                # retry until a good spawn point
                while True:
                    point = random.choice(self.spawn_points)
                    actor = self.spawn_actor(blueprint, point)
                    if actor:
                        self.npc_list.append(actor)
                        break

            except Exception as e:
                if 'collision' not in str(e):
                    print(e)

        for npc in self.npc_list:
            npc.set_autopilot(True)
        print(f'spawned {n_cars} NPCs')
