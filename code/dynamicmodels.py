import numpy as np
from numpy import cos, sin
import numpy.linalg as la
import dataclasses

from typing import List
from typing_extensions import Protocol, runtime


def skew(vector):
    vx, vy, vz = vector
    return np.array([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy,vx,0]])

def rotz(angle):
    return np.array([
            [cos(angle), -sin(angle), 0],
            [sin(angle),  cos(angle), 0],
            [0, 0, 1]])


@runtime
class DynamicModel(Protocol):
    def states(self) -> np.ndarray:
        ...

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
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

        self.rot_body_to_in = np.array([
            [cos(self.yaw), -sin(self.yaw), 0],
            [sin(self.yaw), cos(self.yaw), 0],
            [0, 0, 1]])
        self.vel_in = self.rot_body_to_in @ self.vel_b


    def states(self) -> np.ndarray:
        return np.array([*self.pos_in, *self.vel_b, self.yaw, self.yawrate])

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        forces = inputs[0:3]
        torques = inputs[3:6]
        acc_b = forces/self.mass
        yawacc = torques[2]/self.inertia_z

        return np.array([*self.vel_in, *acc_b, self.yawrate, yawacc])


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

    # Parameter
    time: float = 0
    c1: float = 1.28
    c2: float = 23.99
    c3: float = 0.52
    ks: float = 1.0
    sideslip_smoothing: float = 0.05
    #eps: float = 0.0001
    #B: float 0.05 "Slip r esponse parameter"

    # Dynamic system variables
    omega: float = 0
    steer_angle: float = 0
    steer_angle_dot: float = 0
    sideslip_l: float = 0
    sideslip_s: float = 0

    # Derived variables, computed by compute_derived_variables
    inertia_y: float = 0.0
    inertia_z: float = 0.0
    vel_b: np.ndarray = np.zeros(3)
    vel_in: np.ndarray = np.zeros(3)
    rot_wheel_to_body: np.ndarray = np.eye(3)
    rot_ls_to_wheel: np.ndarray = np.eye(3)
    vel_rot_w: np.ndarray = np.zeros(3)
    vel_rot_ls: np.ndarray = np.zeros(3)
    friction_force: float = 0 # friction force working against drive torque
    friction_torque: float = 0
    force_on_body: np.ndarray = np.zeros(3)
    torque_on_body: np.ndarray = np.zeros(3)
    
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

        self.rot_wheel_to_body = rotz(self.steer_angle)

        self.vel_b = self.vel_body_b + np.cross([0,0,self.yawrate_body],self.pos_b)
        self.beta = np.arctan2(self.vel_b[1], self.vel_b[0])
        self.alpha = self.steer_angle - self.beta # might be good to use smallest signed angle here

        self.rot_ls_to_wheel = rotz(self.alpha)
        rot_body_to_wheel = self.rot_wheel_to_body.T
        rot_wheel_to_ls = self.rot_ls_to_wheel.T
        self.vel_b_ls = rot_wheel_to_ls @ rot_body_to_wheel @ self.vel_b
        # the way alpha is defined, the y- and z-component of the vel_b expressed in
        # LS frame should be zero. So norm(vel_b_ls) == vel_b_ls[0] upto numerical inaccuracy
        # Also norm(vel_b_ls) == norm(vel_b)
        np.testing.assert_almost_equal(la.norm(self.vel_b), self.vel_b_ls[0],
                err_msg="""
                Expected longitudinal velocity component to be equal to the speed.
                Check rotation matrices (especially LS to wheel).
                """)

        self.vel_rot_w = [r*self.omega, 0, 0]
        self.vel_rot_ls = rot_wheel_to_ls @ self.vel_rot_w

        self.force_on_body = np.array([1,0,0])
        self.torque_on_body = np.cross(self.pos_b, self.force_on_body)



    def states(self) -> np.ndarray:
        return np.array([self.omega, self.steer_angle, self.steer_angle_dot, self.sideslip_l, self.sideslip_s])

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        drive_torque, steer_torque = inputs

        omega_dot = (drive_torque - self.friction_torque)/self.inertia_y
        steer_angle_ddot = steer_torque/self.inertia_z

        B = self.sideslip_smoothing
        # TODO: implement sideslip smoothing
        sideslip_l_dot = 0/B
        sideslip_s_dot = 0/B

        return np.array([omega_dot, self.steer_angle_dot, steer_angle_ddot, sideslip_l_dot, sideslip_s_dot])


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




