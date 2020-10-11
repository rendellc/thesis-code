import numpy as np
import numpy.linalg as la

import dataclasses
from dataclasses import field

import utils

def burckhardtfriction(slip_res, c1, c2, c3):
    return c1*(1 - np.exp(-c2*slip_res)) - c3*slip_res

def pacejka_magic(slip):
    #https://se.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html
    # Parameters for dry tarmac
    B,C,D,E = 10, 1.9, 1, 0.97
    return D*np.sin(C*np.arctan(B*slip - E*(B*slip - np.arctan(B*slip))))

def compute_slip(wheel_vel: float, ground_vel: float):
    if np.all(np.isclose([wheel_vel, ground_vel], [0,0])):
        return 0

    vdiff = wheel_vel - ground_vel
    vmax = max(abs(wheel_vel), abs(ground_vel))
    return vdiff/vmax

def dahl_friction_dot(dahl_friction, v, sigma, coulomb_friction):
    return sigma*abs(v)/coulomb_friction*(coulomb_friction*np.sign(v) - dahl_friction)


@dataclasses.dataclass
class WheelModel:
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
    sigma_dahl: float = 4
    #eps: float = 0.0001
    #B: float 0.05 "Slip r esponse parameter"

    # Dynamic system variables
    omega: float = 0
    steer_angle: float = 0
    steer_angle_dot: float = 0
    slip_l: float = 0
    slip_s: float = 0
    friction_dahl_l: float = 0
    friction_dahl_s: float = 0

    # Derived variables, computed by compute_derived_variables
    inertia_y: float = field(init=False,repr=False)
    inertia_z: float = field(init=False,repr=False)
    vel_b: np.ndarray = field(init=False,repr=False)
    vel_in: np.ndarray = field(init=False,repr=False)
    rot_wheel_to_body: np.ndarray = field(init=False,repr=False)
    rot_ls_to_wheel: np.ndarray = field(init=False,repr=False)
    vel_rot_w: np.ndarray = field(init=False,repr=False)
    vel_rot_ls: np.ndarray = field(init=False,repr=False)
    slip_res: float = field(init=False,repr=False)
    mu_res: float = field(init=False,repr=False)
    friction_force: float = field(init=False,repr=False) # friction force working against drive torque
    force_on_body: np.ndarray = field(init=False,repr=False)
    torque_on_body: np.ndarray = field(init=False,repr=False)
    wheel_state: str = field(init=False,repr=False)
    alpha: float = field(init=False,repr=False)
    beta: float = field(init=False,repr=False)

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
        # self.slip_l = np.clip(self.slip_l, -1, 1)
        # self.slip_s = np.clip(self.slip_s, -1, 1)

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

        # test for small velocity values where slip 
        # is zero but numerically infeasible to compute
        # self.slip_l = compute_slip(self.vel_rot_ls[0], self.vel_b_ls[0])
        # self.slip_s = compute_slip(self.vel_rot_ls[1], self.vel_b_ls[1])
        self.slip_res = (self.slip_l**2 + self.slip_s**2)**0.5

        force_z = self.load + utils.weight(self.mass)
        friction_model = "burckhardt"
        if friction_model == "burckhardt":
            self.mu_res = burckhardtfriction(self.slip_res, self.c1, self.c2, self.c3)
            if np.isclose(self.slip_res,0):
                self.mu_l = 0
                self.mu_s = 0
            else:
                self.mu_l = self.slip_l/self.slip_res*self.mu_res
                self.mu_s = self.ks * self.slip_s/self.slip_res*self.mu_res
        if friction_model == "magic":
            self.mu_l = pacejka_magic(self.slip_l)
            self.mu_s = pacejka_magic(self.slip_s)
        if friction_model == "dahl":
            self.mu_l = self.friction_dahl_l/force_z
            self.mu_s = self.friction_dahl_s/force_z

        self.friction_force_ls = np.array([self.mu_l, self.mu_s, 0]) * force_z
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
        #return np.array([self.omega, self.steer_angle, self.steer_angle_dot, self.friction_dahl_l, self.friction_dahl_s])
        return np.array([self.omega, self.steer_angle, self.steer_angle_dot, self.slip_l, self.slip_s])
        # return np.array([self.omega, self.steer_angle, self.steer_angle_dot])

    def derivatives(self, inputs: np.ndarray) -> np.ndarray:
        drive_torque, steer_torque = inputs

        omega_dot = (drive_torque - self.radius*self.friction_force_w[0])/self.inertia_y
        steer_angle_ddot = steer_torque/self.inertia_z

        B = self.slip_smoothing
        vel_rot_l = self.vel_rot_ls[0]
        vel_rot_s = self.vel_rot_ls[1]
        vel_w = self.vel_b_ls[0]
        vel_big = max(abs(vel_rot_l), abs(vel_rot_s), abs(vel_w)) # TODO: more velocities here
        slip_l_dot = -vel_big*self.slip_l/B + (vel_rot_l - vel_w)/B
        slip_s_dot = -vel_big*self.slip_s/B + vel_rot_s/B

        force_z = self.load + utils.weight(self.mass)
        mu_k = 0.7 # kinetic friction between dry rubber and concrete
        force_c = mu_k*force_z
        friction_dahl_l_dot = dahl_friction_dot(self.friction_dahl_l, vel_rot_l - vel_w, self.sigma_dahl, force_c)
        friction_dahl_s_dot = dahl_friction_dot(self.friction_dahl_s, vel_rot_s, self.sigma_dahl, force_c)

        #return np.array([omega_dot, self.steer_angle_dot, steer_angle_ddot, friction_dahl_l_dot, friction_dahl_s_dot])
        return np.array([omega_dot, self.steer_angle_dot, steer_angle_ddot, slip_l_dot, slip_s_dot])
        # return np.array([omega_dot, self.steer_angle_dot, steer_angle_ddot])

    def from_states(self, states):
        """
        Return new instance of dataclass. Do not modifify self.
        """
        omega = states[0]
        steer_angle = states[1]
        steer_angle_dot = states[2]
        # return dataclasses.replace(self, omega=omega, steer_angle=steer_angle,
        #         steer_angle_dot=steer_angle_dot)
        slip_l = states[3]
        slip_s = states[4]
        return dataclasses.replace(self, omega=omega, steer_angle=steer_angle,
                steer_angle_dot=steer_angle_dot,
                slip_l=slip_l, slip_s=slip_s)
        #dahl_l = states[3]
        #dahl_s = states[4]
        #return dataclasses.replace(self, omega=omega, steer_angle=steer_angle,
        #        steer_angle_dot=steer_angle_dot,
        #        friction_dahl_l=dahl_l, friction_dahl_s=dahl_s)

    def from_changes(self, **changes):
        return dataclasses.replace(self, **changes)

