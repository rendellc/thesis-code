import numpy as np

import dataclasses
from dataclasses import field

import utils

@dataclasses.dataclass
class BodyModel:
    """
    Representation of the state of the body at a particular time.
    """
    # Parameters
    mass: float
    width: float
    length: float

    # Dynamic variables
    time: float = 0
    pos_in: np.ndarray = np.zeros(3)
    vel_b: np.ndarray = np.zeros(3)
    yaw: float = 0
    yawrate: float = 0

    # Derived variables, computed by compute_derived_variables
    inertia_z: float = field(init=False,repr=False)
    vel_in: np.ndarray = field(init=False,repr=False)
    rot_body_to_in: np.ndarray = field(init=False,repr=False)

    def __post_init__(self):
        self.compute_derived_variables()

    def compute_derived_variables(self):
        self.inertia_z = 1/12 * self.mass * (self.length**2 + self.width**2)

        self.rot_body_to_in = utils.rotz(self.yaw)
        self.vel_in = self.rot_body_to_in @ self.vel_b


    def states(self) -> np.ndarray:
        return np.array([*self.pos_in, *self.vel_b, self.yaw, self.yawrate])

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        forces = inputs[0:3]
        torques = inputs[3:6]
        acc_b = forces/self.mass
        yawacc = torques[2]/self.inertia_z

        return np.array([*self.vel_in, *acc_b, self.yawrate, yawacc])

    def from_states(self, states):
        pos_in = states[0:3]
        vel_b = states[3:6]
        yaw = states[6]
        yawrate = states[7]
        return dataclasses.replace(self, pos_in=pos_in, vel_b=vel_b, yaw=yaw, yawrate=yawrate)

    def from_changes(self, **changes):
        return dataclasses.replace(self, **changes)


