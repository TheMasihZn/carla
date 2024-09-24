import math

import carla


# noinspection PyChainedComparisons
def location_equal(a: carla.Location, b: carla.Location, location_threshold=1):
    return (a.x >= b.x - location_threshold and
            a.x <= b.x + location_threshold and
            a.y >= b.y - location_threshold and
            a.y <= b.y + location_threshold
            )


def rotation_equal(a: carla.Rotation, b: carla.Rotation, rotation_threshold=30):
    result = True

    result &= (a.yaw - (rotation_threshold / 2)) <= b.yaw <= (a.yaw + (rotation_threshold / 2))
    result &= (a.roll - (rotation_threshold / 2)) <= b.roll <= (a.roll + (rotation_threshold / 2))
    result &= (a.pitch - (rotation_threshold / 2)) <= b.pitch <= (a.pitch + (rotation_threshold / 2))
    return result


def distance_in_route(
        _l1_index_in_path: int,
        _l2_index_in_path: int,
        _path: list
) -> float:
    _route = []
    if _l1_index_in_path < _l2_index_in_path:
        _route = _path[_l1_index_in_path: _l2_index_in_path]
    elif _l1_index_in_path == _l2_index_in_path:
        _route = _path
    else:
        _route = _path[_l1_index_in_path:]
        _route.extend(_path[:_l2_index_in_path])
    d = 0.0
    for i in range(len(_route) - 1):
        l1 = _route[i].location
        l2 = _route[i + 1].location
        d += math.sqrt(
            (l1.x - l2.x) ** 2
            +
            (l1.y - l2.y) ** 2
        )
    return d


def transform_equal(a: carla.Transform,
                    b: carla.Transform,
                    location_threshold=1,
                    rotation_threshold=10):
    return (
            location_equal(a.location, b.location, location_threshold)
            and
            rotation_equal(a.rotation, b.rotation, rotation_threshold)
    )
