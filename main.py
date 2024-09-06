import carla
import pygame
import bridge
from helper import equal
from pov import WorldPOV


def auto_control(_spawn_transform, _bridge: bridge.CarlaBridge):
    pov = WorldPOV(
        _spawn_transform=_spawn_transform,
        _bridge=_bridge,
        size={
            'width': 480,
            'height': 720,
        }
    )

    agent = ShortestPathAgent(pov.player)

    destinations = []
    for lap in range(1):
        destinations.append(
            _bridge.generate_destinations(
                _spawn_transform.location
            )
        )
    agent.set_destination(destinations.pop(0))

    try:
        while True:
            _bridge.world.wait_for_tick()

            if 'break' in pov.on_tick():
                break

            location = pov.player.get_location()

            for hint in _bridge.hints:
                if equal(hint.get_location(), location, 3):
                    hint.destroy()
                    _bridge.hints.remove(hint)

            if agent.done():
                if len(destinations) > 0:
                    agent.set_destination(destinations.pop(0))
                else:
                    break

            control = agent.run_step()
            control.manual_gear_shift = False
            pov.player.apply_control(control)

    finally:
        pygame.quit()
        pov.destroy()


if __name__ == '__main__':
    bridge = bridge.CarlaBridge()
    bridge.spawn_hints()

    start_transform = bridge.map.get_waypoint(
        carla.Location(x=-40.08, y=140.58, z=2.0)
    ).transform

    auto_control(start_transform, _bridge=bridge)

    bridge.destroy_hints()
