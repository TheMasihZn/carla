import random
import carla
import time

random.seed(0)


# noinspection PyArgumentList
class CarlaBridge(object):

    def __init__(self):
        #              \/ \/ \/ the time based on async mode
        self.__tick_dt = 0.02742596348884381030162642100976
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

        self.is_sync = False
        self.__last_tick_time = 0.0

        # todo cannot import these directly.
        self.__setAutopilotCommand = carla.command.SetAutopilot

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

        self.is_sync = False
        print('async')

    def go_sync(self):
        self.traffic_manager.set_synchronous_mode(True)
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = self.__tick_dt
        self.world.apply_settings(self.settings)

        self.is_sync = True
        print('sync')

    def tick(self):
        if not self.is_sync:
            self.world.wait_for_tick()
            return

        self.__last_tick_time = time.time()
        self.world.tick()

    def on_post_tick(self):
        if self.is_sync:
            return
        algorythm_time = time.time() - self.__last_tick_time
        delta_t = self.__tick_dt - algorythm_time
        if delta_t >= 0:
            time.sleep(delta_t)
        else:
            raise Exception("lag: increase Bridge.tick_dt parameter to match the logged ticks per second in async mode")

    def get_tick_dt(self):
        if self.is_sync:
            return self.settings.max_substep_delta_time
        else:
            return self.__tick_dt

    def activate_autopilot(self, actor: carla.Vehicle):

        # todo this is the part you have all the access you need to manipulate anything about the auto pilot
        # \/ \/ \/ \/ \/

        cmd = self.__setAutopilotCommand(actor.id, True, self.traffic_manager.get_port())
        self.client.apply_batch_sync([cmd])

        # todo up to this point you can set you custom autopilot function instead (if possible)
