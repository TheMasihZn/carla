import math
import queue
import threading

import numpy as np
import pygame
from pygame.locals import (KMOD_CTRL, K_ESCAPE, K_q, K_c, WINDOWMOVED)

from cars import Car
import traffic_light_manager


class HUD(object):

    def __init__(self, width, height):
        pygame.init()
        pygame.font.init()
        self.dim = (width, height)
        self.font = pygame.font.Font(pygame.font.get_default_font(), 12)
        self._text_template = []
        self.display = pygame.display.set_mode(
            (width, height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self.thread_wait = False
        self.stop_signal = False
        self.event_thread = threading.Thread(target=self.thread_worker)

        pygame.display.set_caption('Hero POV')
        pygame.init()
        pygame.font.init()
        self.event_thread.start()

    def update_text(
            self,
            _sensor_info,
            _car: Car,
            _traffic_lights
            # _vehicles
    ):

        self._text_template = [
            'Speed:   % 15.0f km/h' % _car.speed,
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (_car.location.x, _car.location.y)),
            'Height:  % 18.0f m' % _car.location.z,
            '',
            ('Throttle:', _car.control.throttle, 0.0, 1.0),
            ('Steer:', _car.control.steer, -1.0, 1.0),
            ('Brake:', _car.control.brake, 0.0, 1.0),
            ('Reverse:', _car.control.reverse),
            ('Hand brake:', _car.control.hand_brake),
            ('Manual:', _car.control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(_car.control.gear, _car.control.gear),
            '',
            ('Traffic lights:', _traffic_lights),
            # 'Number of vehicles in front: % 8d' % (len(_vehicles) + 1)
        ]

    def render(self, _sensor_info):
        frame = _sensor_info[0]['data']
        if frame is not None:
            _image_surface = pygame.surfarray.make_surface(frame)
            if _image_surface is not None:
                self.display.blit(_image_surface, (0, 0))

        info_surface = pygame.Surface((220, self.dim[1]))
        info_surface.set_alpha(100)
        self.display.blit(info_surface, (0, 0))
        v_offset = 4
        bar_h_offset = 100
        bar_width = 100
        for item in self._text_template:
            if v_offset + 18 > self.dim[1]:
                break
            if isinstance(item, list):
                if len(item) > 1:
                    points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                    pygame.draw.lines(self.display, (255, 136, 0), False, points, 2)
                item = None
                v_offset += 18
            elif isinstance(item, tuple):
                box_dim = 6
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (box_dim, box_dim))
                    pygame.draw.rect(self.display, (255, 255, 255), rect, 0 if item[1] else 1)
                elif isinstance(item[1], traffic_light_manager.TrafficLights):
                    dist_light_zip = sorted(
                        zip(
                            item[1].distance_to_targets,
                            item[1].targets
                        ),
                        key=lambda tup: tup[0])
                    for distance, light in dist_light_zip:
                        rect = pygame.Rect((bar_h_offset, v_offset), (box_dim, box_dim))
                        pygame.draw.rect(self.display, light.get_state().__str__(), rect, 0 if item[1] else 1)
                        text = self.font.render("id: %3i  in % 5.0f m" % (light.id, distance), True, (255, 255, 255))
                        self.display.blit(text, (bar_h_offset + 8 + box_dim, v_offset - 3))
                        v_offset += 18
                    v_offset -= (len(item[1].targets) * 18) + (box_dim // 2)
                else:
                    rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, box_dim))
                    pygame.draw.rect(self.display, (255, 255, 255), rect_border, 1)
                    fig = (item[1] - item[2]) / (item[3] - item[2])
                    if item[2] < 0.0:
                        rect = pygame.Rect(
                            (bar_h_offset + fig * (bar_width - box_dim), v_offset + 8), (box_dim, box_dim))
                    else:
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, box_dim))
                    pygame.draw.rect(self.display, (255, 255, 255), rect)
                item = item[0]
            if item:  # At this point has to be a str.
                text = self.font.render(item, True, (255, 255, 255))
                self.display.blit(text, (8, v_offset))
            v_offset += 18
        pygame.display.flip()

    def thread_worker(self):
        while True:
            if self.stop_signal:
                break
            if self.thread_wait:
                continue

            self.thread_wait = True
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.close()
                # elif pygame.key.get_mods() & KMOD_CTRL:
                #     if event.key == K_q or event.key == K_c:
                #         self.stop_signal = True
                #         self.event_queue.put(None)

    def on_tick(self):
        pygame.event.pump()
        self.thread_wait = False

    def close(self):
        if not self.stop_signal:
            self.stop_signal = True
