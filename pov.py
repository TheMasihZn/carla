import carla
import bridge
from lange_invation_sensor import LaneInvasionSensor
from gnss_sensor import GnssSensor
from camera_manager import CameraManager
from hud import HUD
import random

random.seed(0)


class WorldPOV(object):
    def __init__(
            self,
            _bridge: bridge.CarlaBridge,
            _spawn_transform: carla.Transform,
            size: dict
    ):
        self.hud = HUD(size['height'], size['width'])
        self.spawn_transform = _spawn_transform
        self._weather_index = 0

        self.player = self.__spawn_hero(_bridge, _spawn_transform)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)

        self.camera_manager.set_sensor(self.camera_manager.index, notify=False)

    @staticmethod
    def __spawn_hero(
            _bridge: bridge.CarlaBridge,
            _spawn_point: carla.Transform
    ) -> carla.Actor:

        hero_bp = random.choice(_bridge.blueprints)
        hero_bp.set_attribute('role_name', 'hero')

        hero = _bridge.world.spawn_actor(hero_bp, _spawn_point)

        physics_control = hero.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        hero.apply_physics_control(physics_control)

        return hero

    # noinspection PyArgumentList
    def on_tick(self):
        self.hud.set_text_for_tick(
            self.player.get_transform(),
            self.player.get_velocity(),
            self.player.get_control()
        )

        self.camera_manager.render(self.hud.display)
        self.hud.render()

    # noinspection PyArgumentList
    def destroy(self):
        destroy_list = [
            self.camera_manager.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]

        for actor in destroy_list:
            if actor is not None:
                actor.destroy()
                destroy_list.remove(actor)
