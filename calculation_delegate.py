import carla
import csv
import matplotlib.pyplot as plt


# noinspection PyChainedComparisons
def equal(a: carla.Location, b: carla.Location, threshold=1):
    return (a.x >= b.x - threshold and
            a.x <= b.x + threshold and
            a.y >= b.y - threshold and
            a.y <= b.y + threshold
            )

# noinspection PyTypeChecker
def read_route_from_file():
    _route = []
    for line in csv.DictReader(open('route.csv', 'r')):
        rotation = carla.Rotation(float(line['pitch']), float(line['yaw']), float(line['roll']))
        carla_location = carla.Location(float(line['x']), float(line['y']), float(line['z']))
        _route.append(carla.Transform(carla_location, rotation))
    return _route


def draw_route(waypoints, route, all_traffic_lights, chosen_traffic_lights):
    plt.figure(figsize=(7, 7))
    # Extract the (x, y) positions of waypoints
    x_vals = [waypoint.transform.location.x for waypoint in waypoints]
    y_vals = [waypoint.transform.location.y for waypoint in waypoints]
    plt.scatter(x_vals, y_vals, s=1, color='black')

    if route:
        x_route = [transform.location.x for transform in route]
        y_route = [transform.location.y for transform in route]
        plt.scatter(x_route, y_route, s=1, color='yellow')
        plt.scatter(route[0].location.x, route[0].location.y, s=15, color='blue')

    x_all_traffic_lights = [tl.get_location().x for tl in all_traffic_lights]
    y_all_traffic_lights = [tl.get_location().y for tl in all_traffic_lights]
    plt.scatter(x_all_traffic_lights, y_all_traffic_lights, s=30, color='white')

    x_chosen_traffic_lights = [tl.get_location().x for tl in chosen_traffic_lights]
    y_chosen_traffic_lights = [tl.get_location().y for tl in chosen_traffic_lights]
    plt.scatter(x_chosen_traffic_lights, y_chosen_traffic_lights, s=30, color='green')

    # Plot the road network
    plt.title("2D Road-Only Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()
