import numpy as np
import transform as tf

g = 9.81


def burckhardt_friction(slip):
    c1, c2, c3 = 1.28, 23.99, 0.52
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
        wheel_sideslip = np.arctan2(wheel_velocity_hom[1], wheel_velocity_hom[0])
        tire_sideslip = self.angle - wheel_sideslip

        rotational_wheel_speed = self.angular_velocity * self.radius
        wheel_ground_contact_speed = (wheel_velocity_hom[:3] @ wheel_velocity_hom[:3])**0.5

        # Compute slip
        vrcos = rotational_wheel_speed*np.cos(tire_sideslip)
        vrsin = rotational_wheel_speed*np.sin(tire_sideslip)
        vw = wheel_ground_contact_speed

        if vrcos > vw:
            # Braking condition
            slip_l = (vrcos - vw)/vrcos
            slip_s = vrsin/vrcos
        else:
            # Driving condition
            slip_l = (vrcos - vw)/vw
            slip_s = vrsin/vw

        slip_l = slip_l if not np.isnan(slip_l) else 0
        slip_s = slip_s if not np.isnan(slip_s) else 0

        slip_res = (slip_l*slip_l + slip_s*slip_s)**0.5


        return slip_res, slip_l, slip_s, wheel_sideslip

    def friction_force(self, force_z):
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
        friction_force_cg = (tf.rotate_z(wheel_sideslip) @ \
                tf.to_homogeneous(friction_force_veldir))[:3]

        # TODO: Kamm circle?

        return friction_force_cg

    def angular_accel(self, drive, brake, friction_force):
        Iw = self.inertia[1,1] # wheel rotates about y axis
        omega_dot = (drive - brake - self.wheel_radius*friction_force)/Iw
        return omega_dot

    def __str__(self):
        return f"Wheel({self.position}, {self.angle})"

    def __repr__(self):
        return self.__str__()


class Car2D:
    def __init__(self, config):
        self.mass = config["mass"]
        self.width_front = config["width_front"]
        self.width_rear = config["width_rear"]
        self.length_front = config["length_front"]
        self.length_rear = config["length_rear"]

        # Compute moment of inertia matrix
        # pretend its a prism rotating about CG for now
        M = self.mass
        W = (self.width_front + self.width_rear)/2
        L = self.length_front + self.length_rear
        H = config["height"]
        Ixx = (1/12)*M*(H**2 + W**2)
        Iyy = (1/12)*M*(H**2 + L**2)
        Izz = (1/12)*M*(L**2 + W**2)
        self.inertia = np.array([
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]])

        self.position = np.array([0,2,0])  # in inertial frame
        self.velocity = np.array([0,0,0]) # in inertial frame
        # note rpy[0] = rpy[1] = 0 always in this model
        self.rpy = np.array([0,0,0])
        self.rpy_rate = np.array([0,0,0.1])

        lf, lr = self.length_front, self.length_rear
        wf, wr = self.width_front, self.width_rear
        self.wheels = [
                Wheel(np.array([lf, -wf/2, 0]), self, config),
                Wheel(np.array([lf, wf/2, 0]), self, config),
                Wheel(np.array([-lr, -wr/2, 0]), self, config),
                Wheel(np.array([-lr, wr/2, 0]), self, config)]

        self.cg_to_in = tf.translate(self.position) @ tf.rotate_z(-self.rpy[2])
        self.cg_to_in_deriv = tf.translate_deriv(self.velocity) @ tf.rotate_z(-self.rpy[2]) \
                + tf.translate(self.position) @ tf.rotate_z_deriv(-self.rpy[2], -self.rpy_rate[2])

        for wheel in self.wheels:
            wheel_to_in = wheel.wheel_to_cg @ self.cg_to_in
            wheel_force = wheel.friction_force(self.mass*g/len(self.wheels))
            print(wheel, wheel_force)


        self.state = np.concatenate([
            self.position,
            self.velocity,
            self.rpy,
            self.rpy_rate])


