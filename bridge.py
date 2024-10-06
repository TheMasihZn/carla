import random
import carla

random.seed(0)


# noinspection PyArgumentList
class CarlaBridge(object):

    def __init__(self):
        self.client = carla.Client('127.0.0.1', 2000)
        self.world: carla.World = self.client.get_world()
        self.settings: carla.WorldSettings = self.world.get_settings()
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

        self.hints = []

    def spawn_actor(self,
                    bp: carla.ActorBlueprint,
                    point: carla.Transform = None,
                    attach_to=None,
                    attachment_type=carla.AttachmentType.Rigid,
                    destroy_at_the_end=True
                    ) -> carla.Actor:
        if destroy_at_the_end:
            name = bp.get_attribute('role_name').as_str() + '__destroy'
            bp.set_attribute('role_name', name)
        if not point:
            point = random.choice(self.spawn_points)
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

            self.world.wait_for_tick()

    def go_async(self):
        self.traffic_manager.set_synchronous_mode(False)
        self.settings.synchronous_mode = False
        self.settings.fixed_delta_seconds = None
        self.world.apply_settings(self.settings)
        print('async')

    def go_sync(self):
        self.traffic_manager.set_synchronous_mode(True)
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 0.02
        self.world.apply_settings(self.settings)
        print('sync')

