import weakref
import pygame
import numpy as np
import carla
from carla import AttachmentType
from carla import ColorConverter

import bridge


class SensorManager(object):

    def __init__(self, _bridge: bridge.CarlaBridge, player, window_size: dict):
        self.surface = None
        self._player = player
        self.width = window_size['width']
        self.height = window_size['height']

        # todo fix bounding box
        bound_x = 0.5 + self._player.bounding_box.extent.x
        bound_y = 0.5 + self._player.bounding_box.extent.y
        bound_z = 0.5 + self._player.bounding_box.extent.z

        self.sensors = [
            {
                'name': 'Camera RGB',
                'id': 'sensor.camera.rgb',
                'convertion_methode': ColorConverter.Raw,
                'transform': carla.Transform(carla.Location(
                    x=-2.0 * bound_x,
                    y=+0.0 * bound_y,
                    z=+2.0 * bound_z
                ), carla.Rotation(pitch=8.0)),
                'attachment': AttachmentType.SpringArm,
                'data': None,
                'actor': None,
                'n_updates': 0
            },
            {
                'name': 'Lidar (Ray-Cast)',
                'id': 'sensor.lidar.ray_cast',
                'convertion_methode': None,
                'transform': carla.Transform(carla.Location(
                    x=-1.0,
                    y=-1.0 * bound_y,
                    z=+0.4 * bound_z
                ), carla.Rotation()),
                'attachment': AttachmentType.Rigid,
                'data': None,
                'actor': None
            },
            {
                'name': 'Lane Invasion',
                'id': 'sensor.other.lane_invasion',
                'convertion_methode': None,
                'transform': carla.Transform(carla.Location(), carla.Rotation()),
                'attachment': AttachmentType.Rigid,
                'data': None,
                'actor': None
            },
        ]
        self.__spawn_sensors(_bridge)

    def __spawn_sensors(self, _bridge: bridge.CarlaBridge):
        for i, sensor_data in enumerate(self.sensors):
            sensor_bp = _bridge.blueprint_library.find(sensor_data['id'])

            if 'camera' in sensor_data['id']:
                sensor_bp.set_attribute('image_size_x', str(self.height))
                sensor_bp.set_attribute('image_size_y', str(self.width))
            elif 'lidar' in sensor_data['id']:
                sensor_bp.set_attribute('range', '50')

            actor = _bridge.spawn_actor(
                sensor_bp,
                sensor_data['transform'],
                attach_to=self._player,
                attachment_type=sensor_data['attachment']
            )

            # separate every type of sensor deta
            if 'camera' in sensor_data['id']:
                actor.listen(lambda image: self.__process_camera_data(image, 0))
            elif 'lidar' in sensor_data['name']:
                actor.listen(lambda data: self.__process_lidar_data(data, i))
            elif 'lane_invasion' in sensor_data['id']:
                actor.listen(lambda data: self.__process_lane_invasion_data(data, i))

            self.sensors[i]['actor'] = actor

    def __process_camera_data(self, image: carla.Image, sensor_index=-1):
        # image.convert(camera[camera['convertion_methode']])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.sensors[sensor_index]['data'] = array.swapaxes(0, 1)

    def __process_lidar_data(self, data: carla.LidarMeasurement, sensor_info_index=-1):
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
        self.sensors[sensor_info_index]['data'] = lidar_img

    def __process_lane_invasion_data(self, event: carla.LaneInvasionEvent, sensor_info_index=-1):
        if not self:
            return
        self.sensors[sensor_info_index]['data'] = set(x.type for x in event.crossed_lane_markings)
