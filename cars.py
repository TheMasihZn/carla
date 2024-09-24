import csv
import math

import carla
from numpy import random

from bridge import CarlaBridge


class Car(object):
    # noinspection PyTypeChecker
    def __init__(self, _bridge: CarlaBridge, models_file_path, _spawn_point: carla.Location):
        with open(models_file_path, 'r') as file:
            lines = file.readlines()[1:]

        while True:
            car_data = random.choice(lines).split(',')
            try:
                self.actor = _bridge.spawn_actor(
                    _bridge.blueprint_library.filter(car_data[0])[0],
                    _spawn_point
                )
                break
            except IndexError:
                print(f'{car_data[0]} not found')
            finally:
                pass
        self.weight = car_data[1]
        self.max_rpm = car_data[2]
        self.drag = car_data[3]
        self.bounding_box: carla.BoundingBox = None
        self.velocity: carla.Vector3D = None
        self.transform: carla.Transform = None
        self.control: carla.VehicleControl = None
        self.location: carla.Location = None
        self.rotation: carla.Rotation = None

        self.speed = None

        self.update_parameters()

    # noinspection PyArgumentList
    def update_parameters(self):
        self.velocity = self.actor.get_velocity()
        self.transform = self.actor.get_transform()
        self.location = self.transform.location
        self.location.z = 0
        self.rotation = self.transform.rotation
        self.control = self.actor.get_control()
        self.bounding_box = self.actor.bounding_box
        self.speed = 3.6 * math.sqrt(self.velocity.x ** 2 +
                                     self.velocity.y ** 2
                                     # + self.velocity.z ** 2
                                     )

    def inject_control(self, control: carla.VehicleControl):
        self.actor.apply_control(control)
