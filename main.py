import carla
import pygame
import bridge
from pov import WorldPOV
from agent import Agent


if __name__ == '__main__':
    bridge = bridge.CarlaBridge()
    spec: carla.Actor = bridge.world.get_spectator()
    spec.set_transform(
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

    pov = WorldPOV(
        _spawn_transform=spawn_transform,
        _bridge=bridge,
        size={
            'width': 480,
            'height': 720,
        }
    )

    agent = Agent(pov.player, _bridge=bridge)

    # destinations = []
    # for lap in range(1):
    #     for dest in bridge.generate_destinations(spawn_transform.location):
    #         destinations.append(dest)
    #
    # agent.set_destination(destinations.pop(0))

    try:
        while True:
            bridge.world.wait_for_tick()
            if 'break' in pov.on_tick(bridge):
                break

    finally:
        pov.destroy()
        pygame.quit()

