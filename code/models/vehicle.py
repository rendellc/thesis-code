import numpy as np

import dataclasses
from typing import List, TypeVar

from models.body import BodyModel
from models.wheel import WheelModel

import utils

@dataclasses.dataclass
class VehicleModel:
    bs: BodyModel
    wss: List[WheelModel]


    def states(self) -> np.ndarray:
        states = []
        states.extend(self.bs.states())
        for ws in self.wss:
            states.extend(ws.states())

        return np.array(states)

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        """
        inputs = [drive1, steer1, drive2, steer2, ..., drive4, steer4]
        """
        drive_torques = inputs[0::2]
        steer_torques = inputs[1::2]
        derivatives = []

        forces_on_body, torques_on_body = [], []
        for ws in self.wss:
            forces_on_body.append(ws.force_on_body)
            torques_on_body.append(ws.torque_on_body)
        forces_on_body = np.array(forces_on_body)
        torques_on_body = np.array(torques_on_body)
        assert forces_on_body.shape == (4,3)
        assert torques_on_body.shape == (4,3)
        force_on_body = np.sum(forces_on_body,axis=0)
        torque_on_body = np.sum(torques_on_body,axis=0)
        assert force_on_body.shape == (3,)
        assert torque_on_body.shape == (3,)

        body_inputs = np.array([*force_on_body, *torque_on_body])
        derivatives.extend(self.bs.derivatives(body_inputs))

        for i, ws in enumerate(self.wss):
            wheel_inputs = np.array([drive_torques[i],steer_torques[i]])
            derivatives.extend(ws.derivatives(wheel_inputs))

        return np.array(derivatives)

    def from_states(self, states):
        """
        Return new instance of dataclass. Do not modifify self.
        """
        bodystates = states[0:8]
        wheelstatesall = states[8:]
        wheelstates = utils.chunks(wheelstatesall, len(self.wss))

        bs = self.bs.from_states(bodystates)
        wss = [ws.from_states(wheelstate) for ws, wheelstate in zip(self.wss, wheelstates)]

        return dataclasses.replace(self, bs=bs, wss=wss)

    def from_changes(self, **changes):
        return dataclasses.replace(self, **changes)
        



