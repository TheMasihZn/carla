import carla


# noinspection PyChainedComparisons
def equal(a: carla.Location, b: carla.Location, threshold=1):
    return (a.x >= b.x - threshold and
            a.x <= b.x + threshold and
            a.y >= b.y - threshold and
            a.y <= b.y + threshold
            )

def rotation_equal(a: carla.Rotation, b: carla.Rotation, threshold=30):
    a.yaw = a.yaw % 360
    b.yaw = b.yaw % 360
    a.roll = a.roll % 360
    b.roll = b.roll % 360
    a.pitch = a.pitch % 360
    b.pitch = b.pitch % 360


    return (a.x >= b.x - threshold and
            a.x <= b.x + threshold and
            a.y >= b.y - threshold and
            a.y <= b.y + threshold
            )

