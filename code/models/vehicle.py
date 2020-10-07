import numpy as np

import dataclasses
from typing import List, TypeVar

from models.body import BodyModel
from models.wheel import WheelModel

import utils

@dataclasses.dataclass
class VehicleModel:
    body: BodyModel
    wheels: List[WheelModel]


    def states(self) -> np.ndarray:
        states = []
        states.extend(self.body.states())
        for ws in self.wheels:
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
        for ws in self.wheels:
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
        derivatives.extend(self.body.derivatives(body_inputs))

        for i, ws in enumerate(self.wheels):
            wheel_inputs = np.array([drive_torques[i],steer_torques[i]])
            derivatives.extend(ws.derivatives(wheel_inputs))

        return np.array(derivatives)

    def from_states(self, states):
        """
        Return new instance of dataclass. Do not modifify self.
        """
        bodystates = states[0:8]
        wheelstatesall = states[8:]
        wheelstates = utils.chunks(wheelstatesall, len(self.wheels))

        body = self.body.from_states(bodystates)
        wheels = [w.from_states(wheelstate) for w, wheelstate in zip(self.wheels, wheelstates)]

        return dataclasses.replace(self, body=body, wheels=wheels)

    def from_changes(self, **changes):
        return dataclasses.replace(self, **changes)
        



