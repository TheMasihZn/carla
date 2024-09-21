import math
import queue
import threading

import numpy as np
import pygame
from pygame.locals import (KMOD_CTRL, K_ESCAPE, K_q, K_c, WINDOWMOVED)


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

    def set_text_for_tick(
            self,
            _sensor_info,
            _transform,
            _velocity,
            _control,
            _traffic_lights
            # _vehicles
    ):

        self._text_template = [
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(_velocity.x ** 2 + _velocity.y ** 2 + _velocity.z ** 2)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (_transform.location.x, _transform.location.y)),
            'Height:  % 18.0f m' % _transform.location.z,
            '',
            ('Throttle:', _control.throttle, 0.0, 1.0),
            ('Steer:', _control.steer, -1.0, 1.0),
            ('Brake:', _control.brake, 0.0, 1.0),
            ('Reverse:', _control.reverse),
            ('Hand brake:', _control.hand_brake),
            ('Manual:', _control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(_control.gear, _control.gear),
            '',
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
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                    pygame.draw.rect(self.display, (255, 255, 255), rect, 0 if item[1] else 1)
                else:
                    rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                    pygame.draw.rect(self.display, (255, 255, 255), rect_border, 1)
                    fig = (item[1] - item[2]) / (item[3] - item[2])
                    if item[2] < 0.0:
                        rect = pygame.Rect(
                            (bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                    else:
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                    pygame.draw.rect(self.display, (255, 255, 255), rect)
                item = item[0]
            if item:  # At this point has to be a str.
                surface = self.font.render(item, True, (255, 255, 255))
                self.display.blit(surface, (8, v_offset))
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