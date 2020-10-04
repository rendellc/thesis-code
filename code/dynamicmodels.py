import numpy as np
import numpy.linalg as la
import dataclasses

from typing import List, TypeVar
from typing_extensions import Protocol, runtime

import utils


def burckhardtfriction(slip_res, c1, c2, c3):
    return c1*(1 - np.exp(-c2*slip_res)) - c3*slip_res


S = TypeVar("S")

@runtime
class DynamicModel(Protocol[S]):
    def states(self) -> np.ndarray:
        ...

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        ...

    def from_states(self, states: np.ndarray) -> S:
        ...



@dataclasses.dataclass
class BodyState:
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
    inertia_z: float = 1.0
    vel_in: np.ndarray = np.zeros(3)
    rot_body_to_in: np.ndarray = np.eye(3)

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


@dataclasses.dataclass
class WheelState:
    """
    Representation of the state of the body at a particular time.
    """
    mass: float
    radius: float
    width: float
    pos_b: np.ndarray
    vel_body_b: np.ndarray
    yawrate_body: float
    load: float

    # Parameter
    time: float = 0
    c1: float = 1.28
    c2: float = 23.99
    c3: float = 0.52
    ks: float = 1.0
    slip_smoothing: float = 0.05
    #eps: float = 0.0001
    #B: float 0.05 "Slip r esponse parameter"

    # Dynamic system variables
    omega: float = 0
    steer_angle: float = 0
    steer_angle_dot: float = 0
    slip_l: float = 0
    slip_s: float = 0

    # Derived variables, computed by compute_derived_variables
    inertia_y: float = 0.0
    inertia_z: float = 0.0
    vel_b: np.ndarray = np.zeros(3)
    vel_in: np.ndarray = np.zeros(3)
    rot_wheel_to_body: np.ndarray = np.eye(3)
    rot_ls_to_wheel: np.ndarray = np.eye(3)
    vel_rot_w: np.ndarray = np.zeros(3)
    vel_rot_ls: np.ndarray = np.zeros(3)
    slip_res: float = 0
    mu_res: float = 0
    friction_force: float = 0 # friction force working against drive torque
    force_on_body: np.ndarray = np.zeros(3)
    torque_on_body: np.ndarray = np.zeros(3)
    wheel_state: str = ""
    
    alpha: float = 0
    beta: float = 0

    def __post_init__(self):
        self.compute_derived_variables()
    
    def compute_derived_variables(self):
        m = self.mass
        r = self.radius
        w = self.width
        self.inertia_y = 1/2 * m * r**2;
        self.inertia_z = 1/4 * m * r**2 + 1/12 * m * w**2;

        self.rot_wheel_to_body = utils.rotz(self.steer_angle)

        # TODO: this may be non-sensical, but it makes the simulator stable for now
        self.slip_l = np.clip(self.slip_l, -1, 1)
        self.slip_s = np.clip(self.slip_s, -1, 1)

        self.vel_b = self.vel_body_b + np.cross([0,0,self.yawrate_body],self.pos_b)
        self.beta = np.arctan2(self.vel_b[1], self.vel_b[0])
        self.alpha = self.steer_angle - self.beta # might be good to use smallest signed angle here

        self.rot_ls_to_wheel = utils.rotz(self.alpha)
        rot_body_to_wheel = self.rot_wheel_to_body.T
        rot_wheel_to_ls = self.rot_ls_to_wheel.T
        self.vel_b_ls = rot_wheel_to_ls @ rot_body_to_wheel @ self.vel_b
        # the way alpha is defined, the y- and z-component of the vel_b expressed in
        # LS frame should be zero. So norm(vel_b_ls) == vel_b_ls[0] upto numerical inaccuracy
        # Also norm(vel_b_ls) == norm(vel_b)
        np.testing.assert_almost_equal(
                la.norm(self.vel_b), self.vel_b_ls[0],
                err_msg="""
                Expected longitudinal velocity component to be equal to the speed.
                Check rotation matrices (especially LS to wheel).
                """)

        self.vel_rot_w = [r*self.omega, 0, 0]
        self.vel_rot_ls = rot_wheel_to_ls @ self.vel_rot_w

        self.slip_res = (self.slip_l**2 + self.slip_s**2)**0.5
        self.mu_res = burckhardtfriction(self.slip_res, self.c1, self.c2, self.c3)
        if np.isclose(self.slip_res,0):
            self.mu_l = 0
            self.mu_s = 0
        else:
            self.mu_l = self.slip_l/self.slip_res*self.mu_res
            self.mu_s = self.ks * self.slip_s/self.slip_res*self.mu_res

        force_z = self.load + utils.weight(self.mass)
        self.friction_force_ls = np.array([self.mu_l, self.mu_s,0]) * force_z
        self.friction_force_w = self.rot_ls_to_wheel @ self.friction_force_ls

        self.force_on_body = self.rot_wheel_to_body @ self.rot_ls_to_wheel @ self.friction_force_ls
        self.torque_on_body = np.cross(self.pos_b, self.force_on_body)

        # Evaluate wheel state for debugging
        vel_w = self.vel_b_ls[0]
        vel_rot_l = self.vel_rot_ls[0]
        if vel_rot_l > vel_w:
            self.wheel_state = "Driving"
        else:
            self.wheel_state = "Braking"




    def states(self) -> np.ndarray:
        return np.array([self.omega, self.steer_angle, self.steer_angle_dot, self.slip_l, self.slip_s])

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        drive_torque, steer_torque = inputs

        omega_dot = (drive_torque - self.radius*self.friction_force_w[0])/self.inertia_y
        steer_angle_ddot = steer_torque/self.inertia_z

        B = self.slip_smoothing
        # TODO: implement slip smoothing
        vel_rot_l = self.vel_rot_ls[0]
        vel_rot_s = self.vel_rot_ls[1]
        vel_w = self.vel_b_ls[0]
        vel_big = max(vel_rot_l, vel_w)
        slip_l_dot = -abs(vel_big)*self.slip_l/B + (vel_rot_l - vel_w)*utils.sign(vel_big)/B
        slip_s_dot = -abs(vel_big)*self.slip_s/B + vel_rot_s*utils.sign(vel_big)/B

        return np.array([omega_dot, self.steer_angle_dot, steer_angle_ddot, slip_l_dot, slip_s_dot])

    def from_states(self, states):
        """
        Return new instance of dataclass. Do not modifify self.
        """
        omega = states[0]
        steer_angle = states[1]
        steer_angle_dot = states[2]
        slip_l = states[3]
        slip_s = states[4]
        return dataclasses.replace(self, omega=omega, steer_angle=steer_angle,
                steer_angle_dot=steer_angle_dot,
                slip_l=slip_l, slip_s=slip_s)


@dataclasses.dataclass(frozen=True)
class VehicleState:
    bs: BodyState
    wss: List[WheelState]


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





