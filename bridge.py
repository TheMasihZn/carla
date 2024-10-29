import random
import carla
import time

import numpy as np

random.seed(0)


# noinspection PyArgumentList
class CarlaBridge(object):

    def __init__(self):
        #              \/ \/ \/ the time based on async mode (1 / average async tps)
        self.tick_dt = 1 / 60
        self.async_hardware_factor = 1.0860360178222745
        self.sync_hardware_factor = 0.6352310657348632
        self._dt_list = []
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

        self.is_async = True
        self.__last_tick_time = -1
        self.__tps_error = 0

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
        print('destroying actors...')
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
        self.settings.fixed_delta_seconds = self.tick_dt / self.async_hardware_factor
        self.settings.max_substeps = 10
        self.settings.max_substep_delta_time = self.settings.fixed_delta_seconds / (self.settings.max_substeps - 2)
        self.world.apply_settings(self.settings)

        self.is_async = True
        print('async')

    def go_sync(self):
        self.settings.fixed_delta_seconds = self.tick_dt / self.sync_hardware_factor
        self.settings.max_substeps = 10
        self.settings.max_substep_delta_time = self.settings.fixed_delta_seconds / (self.settings.max_substeps - 2)
        self.settings.synchronous_mode = True
        self.world.apply_settings(self.settings)
        self.traffic_manager.set_synchronous_mode(True)

        self.is_async = False
        print('sync')

    def tick(self):
        now = time.time()
        if self.__last_tick_time == -1:
            self.__last_tick_time = now
        if self.is_async:
            self.world.wait_for_tick()
            dt = now - self.__last_tick_time
            if self._dt_list is not None:
                self._dt_list.append(dt)
                if len(self._dt_list) < 100:
                    self._dt_list.append(dt)
                else:
                    print("async average tps: %f" % np.average(np.array(self._dt_list)))
                    self._dt_list = None
            self.__last_tick_time = now
        else:
            self.world.tick()

    def post_tick(self):
        if self.is_async:
            return
        algorythm_time = time.time() - self.__last_tick_time
        delta_t = self.tick_dt - algorythm_time
        if delta_t >= 0:
            time.sleep(delta_t)
            self.__tps_error = 0
        # else:
        #     self.__tps_error += 1
        #     if self.__tps_error > 50:
        #         raise Exception("lag: increase Bridge.tick_dt to match the system\n" +
        #                         "average ticks per second in logged in async mode")
        self.__last_tick_time = time.time()

    def get_tick_dt(self):
        if self.is_async:
            return self.settings.fixed_delta_seconds
        else:
            return self.tick_dt

    def activate_autopilot(self, actor: carla.Vehicle):

        # todo this is the part you have all the access you need to manipulate anything about the auto pilot
        # \/ \/ \/ \/ \/

        cmd = self.__setAutopilotCommand(actor.id, True, self.traffic_manager.get_port())
        self.client.apply_batch_sync([cmd])

        # todo up to this point you can set you custom autopilot function instead (if possible)
