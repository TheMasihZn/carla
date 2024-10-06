import pygame
import bridge

import carla
from carla import AttachmentType
from carla import ColorConverter
from carla.libcarla import TrafficLightState

from numpy import random
from cars import Car
from router import Router
from pov import POV
from traffic_light_manager import TrafficLights


def spawn_hero(
        _bridge: bridge.CarlaBridge,
        _spawn_point: carla.Transform
) -> carla.Actor:
    hero_bp = random.choice(_bridge.vehicle_blueprints)
    hero_bp.set_attribute('role_name', 'hero')

    hero = _bridge.spawn_actor(hero_bp, _spawn_point)

    physics_control = hero.get_physics_control()
    physics_control.use_sweep_wheel_collision = True
    hero.apply_physics_control(physics_control)

    return hero


if __name__ == '__main__':
    bridge = bridge.CarlaBridge()

    bridge.spectator.set_transform(
        carla.Transform(
            carla.Location(
                x=-70.172501,
                y=128.424377,
                z=99.078232
            ), carla.Rotation(
                pitch=-56.843441,
                yaw=-51.880329,
                roll=0.000022
            )
        )
    )
    # bridge.spectator.set_transform(
    #     carla.Transform(
    #         carla.Location(x=104.854881, y=36.462254, z=3.997350),
    #         carla.Rotation(pitch=-9.908937, yaw=-49.531647, roll=0.000097))
    # )

    traffic_lights = {
        13: {
            'initial_state': TrafficLightState.Green,
            'green_time': 15.0,
            'yellow_time': 5.0,
            'red_time': 20.0,
        },
        11: {
            'initial_state': TrafficLightState.Red,
            'green_time': 10.0,
            'yellow_time': 5.0,
            'red_time': 25.0,
        },
        20: {
            'initial_state': TrafficLightState.Red,
            'green_time': 12.0,
            'yellow_time': 4.0,
            'red_time': 24.0,
        },
    }

    spawn_transform = bridge.map.get_waypoint(
        carla.Location(x=-40.08, y=140.58, z=2.0)
    ).transform
    spawn_transform.location.z = 2.0

    pov = None
    try:

        ego = Car(
            bridge,
            'models.csv',
            spawn_transform
        )

        bound_x = 0.5 + ego.bounding_box.extent.x
        bound_y = 0.5 + ego.bounding_box.extent.y
        bound_z = 0.5 + ego.bounding_box.extent.z

        sensor_list = [
            {
                'name': 'Camera RGB',
                'id': 'sensor.camera.rgb',
                'transform': carla.Transform(carla.Location(
                    x=-2.0 * bound_x,
                    y=+0.0 * bound_y,
                    z=+2.0 * bound_z
                ), carla.Rotation(pitch=8.0)),
                'attachment': AttachmentType.SpringArm,
                'n_updates': 0
            },
            {
                'name': 'Lidar (Ray-Cast)',
                'id': 'sensor.lidar.ray_cast',
                'transform': carla.Transform(carla.Location(
                    x=-1.0,
                    y=-1.0 * bound_y,
                    z=+0.4 * bound_z
                ), carla.Rotation()),
                'attachment': AttachmentType.Rigid,
            },
            {
                'name': 'Lane Invasion',
                'id': 'sensor.other.lane_invasion',
                'transform': carla.Transform(carla.Location(), carla.Rotation()),
                'attachment': AttachmentType.Rigid,
            },
        ]

        window_size = {
            'width': 480,
            'height': 720,
        }

        router = Router(bridge, route_file_path='route.csv', spawn_hints=True)
        tl_manager = TrafficLights(_bridge=bridge, _router=router, _initial_settings=traffic_lights)
        pov = POV(
            _spawn_transform=spawn_transform,
            _bridge=bridge,
            _router=router,
            _car=ego,
            _sensor_list=sensor_list,
            _traffic_light_manager=tl_manager,
            _window_size=window_size
        )

        bridge.spawn_teraffic(20)

        # bridge.go_sync()

        while True:
            # bridge.world.tick()
            bridge.world.wait_for_tick()

            if 'break' in pov.on_tick(bridge):
                break

    except KeyboardInterrupt:
        pass
    # except Exception as e:
    #     print(e)
    finally:
        # bridge.go_async()
        if pov:
            pov.close()
        pygame.quit()
        print('destroying actors...')
        bridge.delete_created_actors()
