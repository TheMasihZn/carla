import carla
import pygame
import bridge
from pov import WorldPOV
from agent import Agent


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

    spawn_transform = bridge.map.get_waypoint(
        carla.Location(x=-40.08, y=140.58, z=2.0)
    ).transform

    spawn_transform.location.z = 2.0

    try:
        bridge.spawn_teraffic(40)
        # pov = WorldPOV(
        #     _spawn_transform=spawn_transform,
        #     _bridge=bridge,
        #     size={
        #         'width': 480,
        #         'height': 720,
        #     }
        # )
        #
        # agent = Agent(pov.player, _bridge=bridge)

        # while True:
        #     bridge.world.wait_for_tick()
        #
        #     if 'break' in pov.on_tick(bridge):
        #         break

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        print('destroying actors...')
        bridge.delete_created_actors()

