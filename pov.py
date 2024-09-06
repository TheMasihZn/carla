import carla
import bridge
from sensor_manager import SensorManager
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
        self.sensor_manager = SensorManager(self.player, self.hud, size)

    @staticmethod
    def __spawn_hero(
            _bridge: bridge.CarlaBridge,
            _spawn_point: carla.Transform
    ) -> carla.Actor:
        """
        :return: hero actor
        """
        hero_bp = random.choice(_bridge.blueprints)
        hero_bp.set_attribute('role_name', 'hero')

        hero = _bridge.world.spawn_actor(hero_bp, _spawn_point)

        physics_control = hero.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        hero.apply_physics_control(physics_control)

        return hero

    # noinspection PyArgumentList
    def on_tick(self):
        """
        :return: 'break' if break event
        """
        self.hud.set_text_for_tick(
            self.player.get_transform(),
            self.player.get_velocity(),
            self.player.get_control()
        )

        self.hud.render()

        if self.hud.return_key_pressed():
            return 'break'

    # noinspection PyArgumentList
    def destroy(self):
        destroy_list = [
            *self.sensor_manager.sensors,
            self.player]

        for actor in destroy_list:
            if actor is not None:
                actor.destroy()
                destroy_list.remove(actor)
