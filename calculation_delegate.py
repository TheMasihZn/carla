import carla


# noinspection PyChainedComparisons
def equal(a: carla.Location, b: carla.Location, threshold=1):
    return (a.x >= b.x - threshold and
            a.x <= b.x + threshold and
            a.y >= b.y - threshold and
            a.y <= b.y + threshold
            )

