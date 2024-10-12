import math

import carla
import numpy as np

import bridge
import router


# noinspection PyTypeChecker
class Car:
    def __init__(self, actor: carla.Actor, data: list):
        self.actor = actor
        self.name: str = data[0]
        self.mass: float = float(data[1])
        self.max_rpm: float = float(data[2])
        self.bounding_box: carla.BoundingBox = None
        self.velocity: carla.Vector3D = None
        self.acceleration: float = 0.0
        self.transform: carla.Transform = None
        self.forward: carla.Vector3D = None
        self.location: carla.Location = None
        self.rotation: carla.Rotation = None

        self.speed = None
        self.speed_mps = None

        self.i_on_path = -1

        self._update_parameters()

    # noinspection PyArgumentList
    def _update_parameters(self):
        self.velocity = self.actor.get_velocity()
        self.transform = self.actor.get_transform()
        self.location = self.transform.location
        self.location.z = 0
        self.rotation = self.transform.rotation
        self.forward = self.rotation.get_forward_vector()
        self.bounding_box = self.actor.bounding_box
        _new_speed_mps = math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2)
        if self.speed_mps:
            if _new_speed_mps != self.speed_mps:
                self.acceleration = (self.speed_mps + _new_speed_mps) / 2
        self.speed_mps = _new_speed_mps
        self.speed = 3.6 * self.speed_mps


class Ego(Car):
    # noinspection PyTypeChecker
    def __init__(self, actor: carla.Actor, data: list):
        super().__init__(actor, data)

        phys = self.actor.get_physics_control()
        phys.use_sweep_wheel_collision = True
        actor.apply_physics_control(phys)

        self.mass = phys.mass
        self.gear = None
        self.drag = phys.drag_coefficient
        self.max_rpm = phys.max_rpm
        self.damping_rate_zero_throttle_clutch_engaged = phys.damping_rate_zero_throttle_clutch_engaged
        self.damping_rate_full_throttle = phys.damping_rate_full_throttle
        self.should_stop_in = None
        self.control: carla.VehicleControl = None

        self._update_parameters()

    def update_parameters(self):
        super()._update_parameters()
        self.control = self.actor.get_control()
        self.gear = self.control.gear
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


class NPC(Car):
    def __init__(self, actor: carla.Actor, data: list):
        super().__init__(actor, data)
        self.distance_ego_to_car = -1.

    def update_parameters(self):
        super()._update_parameters()
