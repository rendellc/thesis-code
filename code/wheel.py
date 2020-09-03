import numpy as np
import transform as tf


g = 9.81


def burckhardt_friction(slip):
    c1, c2, c3 = 1.28, 23.99, 0.52 # dry asphalt
    return c1*(1 - np.exp(-c2*slip)) - c3*slip


class Wheel:
    def __init__(self, position, vehicle, config):
        self.position = position 
        self.vehicle = vehicle
        self.angle = 0*np.pi/180
        self.angular_velocity = 0
        self.mass = config["wheel_mass"]
        self.radius = config["wheel_radius"]
        self.width = config["wheel_width"]

        M = self.mass
        R = self.radius
        W = self.width
        Ixx = Izz = (1/12)*self.mass*(3*R**2 + W**2)
        Iyy = (1/2)*M*R**2
        self.inertia = np.array([
            [Ixx,0,0],
            [0,Iyy,0],
            [0,0,Izz]])

        self.wheel_to_cg = tf.translate(self.position) @ tf.rotate_z(self.angle)

    def slips(self):
        pos_hom = tf.to_homogeneous(self.position)
        vehicle_vel_hom = tf.to_homogeneous(self.vehicle.velocity)
        wheel_velocity_hom = self.vehicle.cg_to_in_deriv @ pos_hom + vehicle_vel_hom 
        wheel_sideslip = np.arctan2(wheel_velocity_hom[0], wheel_velocity_hom[1])
        tire_sideslip = self.angle - wheel_sideslip

        rotational_wheel_speed = self.angular_velocity * self.radius
        wheel_ground_contact_speed = (wheel_velocity_hom[:3] @ wheel_velocity_hom[:3])**0.5

        # Compute slip
        vrcos = rotational_wheel_speed*np.cos(tire_sideslip)
        vrsin = rotational_wheel_speed*np.sin(tire_sideslip)
        vw = wheel_ground_contact_speed

        if vrcos > vw:
            # Driving condition
            slip_l = (vrcos - vw)/vrcos
            slip_s = vrsin/vrcos
        else:
            # Braking condition
            slip_l = (vrcos - vw)/vw
            slip_s = vrsin/vw

        slip_l = slip_l if not np.isnan(slip_l) else 0
        slip_s = slip_s if not np.isnan(slip_s) else 0
        slip_res = (slip_l*slip_l + slip_s*slip_s)**0.5

        # store in state
        self.slip_l = slip_l
        self.slip_s = slip_s
        self.slip_res = slip_res
        self.wheel_sideslip = wheel_sideslip

        return slip_res, slip_l, slip_s, wheel_sideslip

    def compute_friction_force(self, force_z):
        """
        Compute force wheel exerts on chassis. Force is represented in cg frame.
        TODO: Force z will be affect by load transfer. Ignore for now.
        Return force and update it in state
        """

        slip_res, slip_l, slip_s, wheel_sideslip = self.slips()
        mu_res = burckhardt_friction(slip_res)
        ks = 1
        mu_l = mu_res * slip_l/slip_res
        mu_s = mu_res * ks * slip_s/slip_res 

        # deal with nan
        mu_l = mu_l if not np.isnan(mu_l) else 0
        mu_s = mu_s if not np.isnan(mu_s) else 0

        # compute friction force in cg
        friction_force_veldir = np.array([mu_l*force_z, mu_s*force_z, 0])
        friction_force_cg = tf.rotate_z(wheel_sideslip)[:3,:3] @ friction_force_veldir

        # TODO: Kamm circle?

        assert not np.any(np.isnan(friction_force_cg)), f"Force has nan: {friction_force_cg}"
        self.force_cg = friction_force_cg

    def compute_derivatives(self):
        drive = 0
        Iw = self.inertia[1,1] # wheel rotates about y axis
        self.cg_to_wheel = tf.inv(self.wheel_to_cg)
        self.force_w = self.cg_to_wheel[:3,:3] @ self.force_cg
        self.angular_accel = (drive - self.radius*self.force_w[0])/Iw

    def step_forward(self, stepsize):
        self.angular_velocity += stepsize*self.angular_accel

    def get_simulatable_children(self):
        return []


    def __str__(self):
        return f"Wheel({self.position}, {self.angle})"

    def __repr__(self):
        return self.__str__()


