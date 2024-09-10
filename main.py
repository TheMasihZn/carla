import carla
import pygame
import bridge
from pov import WorldPOV
from agent import ShortestPathAgent


if __name__ == '__main__':
    bridge = bridge.CarlaBridge()

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

    agent = ShortestPathAgent(pov.player)

    destinations = []
    for lap in range(1):
        for dest in bridge.generate_destinations(spawn_transform.location):
            destinations.append(dest)

    agent.set_destination(destinations.pop(0))

    try:
        while True:
            bridge.world.wait_for_tick()
            if 'break' in pov.on_tick():
                break

            if agent.done():
                if len(destinations) > 0:
                    agent.set_destination(destinations.pop(0))
                else:
                    break

            control = agent.run_step()
            control.manual_gear_shift = False
            pov.player.apply_control(control)

    finally:
        pov.destroy()
        pygame.quit()

