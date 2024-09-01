import weakref
import pygame
import numpy as np
import carla
from carla import AttachmentType
from carla import ColorConverter

import bridge


class CameraManager(object):

    # noinspection PyArgumentList
    def __init__(self, _bridge: bridge.CarlaBridge, player, size: dict):
        self.surface = None
        self._player = player
        self.width = size['width']
        self.height = size['height']

        bound_x = 0.5 + self._player.bounding_box.extent.x
        bound_y = 0.5 + self._player.bounding_box.extent.y
        bound_z = 0.5 + self._player.bounding_box.extent.z

        self.sensors = self.__spawn_sensors(_bridge, sensor_spawn_data=[
            {
                'name': 'Camera RGB',
                'id': 'sensor.camera.rgb',
                'convertion_methode': ColorConverter.Raw,
                'transform': carla.Transform(carla.Location(
                    x=-2.0 * bound_x,
                    y=+0.0 * bound_y,
                    z=2.0 * bound_z
                ), carla.Rotation(pitch=8.0)),
                'attachment': AttachmentType.SpringArmGhost
            }, {
                'name': 'Lidar (Ray-Cast)',
                'id': 'sensor.lidar.ray_cast',
                'convertion_methode': None,
                'transform': carla.Transform(carla.Location(
                    x=-1.0,
                    y=-1.0 * bound_y,
                    z=0.4 * bound_z
                ), carla.Rotation()),
                'attachment': AttachmentType.Rigid,
            }
        ])

    def __spawn_sensors(self, _bridge: bridge.CarlaBridge, sensor_spawn_data: list) -> list:
        sensor_list = []
        for sensor_data in sensor_spawn_data:
            sensor_bp = _bridge.blueprints.find(sensor_data['id'])

            if sensor_data['id'].startswith('sensor.camera'):
                sensor_bp.set_attribute('image_size_x', self.width)
                sensor_bp.set_attribute('image_size_y', self.height)
            elif sensor_data['id'].startswith('sensor.lidar'):
                sensor_bp.set_attribute('range', '50')
                sensor_data.append(sensor_bp)

            actor = self._player.get_world().spawn_actor(
                sensor_bp,
                sensor_data['transform'],
                attach_to=self._player,
                attachment_type=sensor_data['attachment'])

            # separate every type of sensor deta
            weak_self = weakref.ref(self)
            if sensor_data['id'].startswith('sensor.camera'):
                actor.listen(
                    lambda image: CameraManager.__process_camera_data(
                        weak_self,
                        image,
                        sensor_data['convertion_methode']
                    )
                )
            elif sensor_data['id'].startswith('sensor.lidar'):
                actor.listen(
                    lambda data: CameraManager.__process_lidar_data(
                        weak_self,
                        data
                    )
                )
            sensor_list.append(actor)
        return sensor_list

    @staticmethod   # self = weak_self
    def __process_camera_data(self, image: carla.Image, convertion_methode=None):
        if convertion_methode:
            image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    @staticmethod   # self = weak_self
    def __process_lidar_data(self, data: carla.LidarMeasurement):
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(self.width, self.height) / 100.0
        lidar_data += (0.5 * self.width, 0.5 * self.height)
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.width, self.height, 3)
        lidar_img = np.zeros(lidar_img_size)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        self.surface = pygame.surfarray.make_surface(lidar_img)

