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
        self.name = car_data[0]
        self.max_rpm = float(car_data[2])
        phys = self.actor.get_physics_control()

        self.mass = phys.mass
        self.drag = phys.drag_coefficient
        self.max_rpm = phys.max_rpm
        self.damping_rate_zero_throttle_clutch_engaged = phys.damping_rate_zero_throttle_clutch_engaged
        self.damping_rate_full_throttle = phys.damping_rate_full_throttle
        self.should_stop_in = None
        self.bounding_box: carla.BoundingBox = None
        self.velocity: carla.Vector3D = None
        self.transform: carla.Transform = None
        self.forward: carla.Vector3D = None
        self.control: carla.VehicleControl = None
        self.location: carla.Location = None
        self.rotation: carla.Rotation = None
        self.__i_in_path = 0

        self.speed = None
        self.speed_mps = None

        self.update_parameters()

    # noinspection PyArgumentList
    def update_parameters(self):
        self.velocity = self.actor.get_velocity()
        self.transform = self.actor.get_transform()
        self.location = self.transform.location
        self.location.z = 0
        self.rotation = self.transform.rotation
        self.forward = self.rotation.get_forward_vector()
        self.control = self.actor.get_control()
        self.bounding_box = self.actor.bounding_box
        self.speed_mps = math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2)
        self.speed = 3.6 * self.speed_mps
        self.should_stop_in = (
                (self.speed
                 / (
                         self.damping_rate_zero_throttle_clutch_engaged -
                         self.damping_rate_full_throttle -
                         self.drag
                 )
                 ) +
                self.bounding_box.extent.x / 2)

    def inject_control(self, control: carla.VehicleControl):
        self.actor.apply_control(control)

