from collections import deque
import math
import numpy as np
import carla


class PIDController(object):
    def __init__(self):
        self.accelerator = PIDAcceleration(target_speed=40)
        self.steer = PIDSteering()

    def get_new_control(
            self,
            _previous_control: carla.VehicleControl,
            _should_stop: bool,
            _speed,
            _dest: carla.Location,
            _now_at: carla.Location,
            _forward_v
    ):
        if not _should_stop:
            throttle_adjustment = self.accelerator.on_tick(speed=_speed)
            _previous_control.throttle += throttle_adjustment

        steer_adjustment = self.steer.on_tick(_dest, _now_at=_now_at, forward_v=_forward_v)

        _previous_control.steer += steer_adjustment
        # control.hand_brake = False
        # control.manual_gear_shift = False

        return _previous_control


class PIDAcceleration(object):
    def __init__(self, target_speed):
        self._target_speed = target_speed
        self._k_p = 1.0
        self._k_i = 0.0
        self._k_d = 0.0
        self._dt = 0.03
        self._error_buffer = deque(maxlen=10)

    def on_tick(self, speed):
        error = self._target_speed - speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDSteering(object):
    def __init__(self):
        self._k_p = 1.0
        self._k_i = 0.0
        self._k_d = 0.0
        self._dt = 0.03
        self._offset = 0.0
        self._e_buffer = deque(maxlen=10)

    def on_tick(self, _dest: carla.Location, _now_at: carla.Location, forward_v: carla.Vector3D):
        forward_v = np.array([forward_v.x, forward_v.y, 0.0])

        w_vec = np.array([_dest.x - _now_at.x,
                          _dest.y - _now_at.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(forward_v)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, forward_v) / wv_linalg, -1.0, 1.0))
        _cross = np.cross(forward_v, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)
