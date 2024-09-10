import carla
import bridge
from sensor_manager import SensorManager
from hud import HUD
import random
from calculation_delegate import equal

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
        self.target_lights = _bridge.target_lights

        self.player = self.__spawn_hero(_bridge, _spawn_transform)
        self.sensor_manager = SensorManager(_bridge, self.player, size)
        self.hints = self.spawn_hints(_bridge)

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

    @staticmethod
    def spawn_hints(
            _bridge: bridge.CarlaBridge
    ):
        _hints = []
        _hint_bp = _bridge.blueprint_library.filter('static.prop.ironplank')[0]
        for t in _bridge.route:
            t.location.z = 0
            _hint = _bridge.world.spawn_actor(_hint_bp, t)
            _hints.append(_hint)

        return _hints

    # noinspection PyArgumentList
    def on_tick(self):
        """
        :return: 'break' if break event
        """
        location = self.player.get_location()

        for hint in self.hints:
            if equal(hint.get_location(), location, 3):
                hint.destroy()
                self.hints.remove(hint)

        self.hud.set_text_for_tick(
            self.sensor_manager.sensors,
            self.player.get_transform(),
            self.player.get_velocity(),
            self.player.get_control(),
            self.target_lights
        )

        self.hud.render(self.sensor_manager.sensors)

        if self.hud.return_key_pressed():
            return 'break'
        else:
            return ''

    # noinspection PyArgumentList
    def destroy(self):

        destroy_list = [
            *[sensor['actor'] for sensor in self.sensor_manager.sensors],
            self.player]
        for actor in destroy_list:
            if actor is not None:
                done = actor.destroy()
                if done:
                    destroy_list.remove(actor)

        while len(self.hints) > 0:
            for hint in self.hints:
                if hint:
                    done = hint.destroy()
                    if done:
                        self.hints.remove(hint)
