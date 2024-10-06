import numpy as np

from bridge import CarlaBridge
from cars import Ego, NPC

import carla


class CarManager:
    # noinspection PyTypeChecker
    def __init__(
            self,
            _bridge: CarlaBridge,
            _models_file_path: str,
            _ego_spawn_point: carla.Transform,
            _initial_traffic: int = 0,
    ):
        self.car_blueprints = {}

        with open(_models_file_path, 'r') as file:
            lines = file.readlines()[1:]
            for line_str in lines:
                line = line_str.split(',')
                self.car_blueprints[line] = _bridge.blueprint_library.filter(line[0])[0]

        self.npc_list = []
        self.ego: Ego = None

        if not self.ego:
            data = np.random.choice(self.car_blueprints)
            blueprint = self.car_blueprints[data]
            blueprint.set_attribute('role_name', 'hero')
            actor = self.__spawn_car_actor(_bridge, blueprint, _ego_spawn_point)
            self.ego = Ego(actor=actor, data=data)
            print('Ego spawned')

        while len(self.npc_list) < _initial_traffic:
            data = np.random.choice(self.car_blueprints)
            blueprint = self.car_blueprints[data]
            blueprint.set_attribute('role_name', 'npc')
            actor = self.__spawn_car_actor(_bridge, blueprint)
            actor.set_autopilot(True)
            self.npc_list.append(NPC(actor=actor, data=data))
        print(f'{_initial_traffic} NPCs spawned')


    @staticmethod
    def __spawn_car_actor(_bridge, _blueprint, _spawn_point=None):
        while True:
            try:
                actor = _bridge.spawn_actor(_blueprint, _spawn_point)
                break
            except Exception as e:
                if 'collision' not in str(e):
                    print(e)
        return actor

    def on_tick(self):
        self.ego.update_parameters()
        for npc in self.npc_list:
            npc.update_parameters()
