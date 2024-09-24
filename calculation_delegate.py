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
        _li_index_in_path: int,
        _path: list) -> float:
    d = 0.0
    for transform in self.path:
        if location_equal()

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
