import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

class VehicleAnimation:
    def __init__(self, position, length, width, size=30, ax=None):
        self.length = length
        self.width = width
        self.wheel_thickness = 0.4
        self.wheel_radius = 0.5

        if ax is None:
            fig, ax = plt.subplots(1,1)
        self.ax = ax
        self.ax.set_aspect("equal")
        # automate this
        self.ax.set_xlim(-size/2, size/2)
        self.ax.set_ylim(-size/2, size/2)
        self.ax.set_xlabel(r"$x (m)$")
        self.ax.set_ylabel("y (m)")
        self.ax.grid(True)

        # draw chassis
        x,y = position[0], position[1]
        xr = x - length/2
        yr = y - width/2
        # self.chassis = mpl.patches.Rectangle((xr, yr), length, width)
        L = self.length
        B = self.width
        D = 2*self.wheel_radius
        xy = np.array([
            [L/2,0],
            [L/2,B/2],
            [L/2-D,B/2],
            [L/2-D,B/4],
            [-L/2+D,B/2],
            [-L/2,B/2],
            [-L/2,B/4],
            [-L/2+D,B/4],
            [L/4,0],
            [-L/2+D,-B/4],
            [-L/2,-B/4],
            [-L/2,-B/2],
            [-L/2+D,-B/2],
            [L/2-D,-B/4],
            [L/2-D,-B/2],
            [L/2,-B/2]])
        self.chassis = mpl.patches.Polygon(xy)

        self.ax.add_patch(self.chassis)

        # draw wheels
        self.corners = [
                (length/2, width/2),  # front left
                (-length/2, width/2), # rear left
                (-length/2, -width/2),  # rear right
                (length/2, -width/2)]   # front right
        self.wheels = []
        for corner in self.corners:
            xc, yc = corner
            xr = x + xc - self.wheel_radius
            yr = y + yc - self.wheel_thickness/2

            rect = mpl.patches.Rectangle((xr,yr), 2*self.wheel_radius, self.wheel_thickness, color="red")
            self.ax.add_patch(rect)
            self.wheels.append(rect)


        plt.show(block=False)

    def update(self, t, position, chassis_angle, wheel_angles, dt=1e-10):
        # force_total = np.sum(np.abs(wheel_forces))
        # force_ratios = wheel_forces/(force_total+1e-8)

        self.ax.set_title(f"time {t:.4f}")

        x, y = position[0], position[1]
        xr = x - self.length/2
        yr = y - self.width/2
        # xy_rect = (xr, yr) # bottom left corner

        # transformations
        t_trans = mpl.transforms.Affine2D().translate(x,y)
        t_rot = mpl.transforms.Affine2D().rotate(chassis_angle)
        t_chassis = t_rot + t_trans + self.ax.transData
        self.chassis.set_transform(t_chassis)

        c, s = np.cos(chassis_angle), np.sin(chassis_angle)
        rot_chassis = np.array([
            [c, -s],
            [s, c]])
        for corner, wheel, wheel_angle in zip(self.corners, self.wheels, wheel_angles):
            xc, yc = corner

            wheel_vec = rot_chassis @ np.array([xc, yc])
            xw = x + wheel_vec[0]
            yw = y + wheel_vec[1]
            xr = xw - self.wheel_radius
            yr = yw - self.wheel_thickness/2

            wheel.set_xy((xr,yr))
            t_rot = mpl.transforms.Affine2D().rotate_around(xw, yw, chassis_angle + wheel_angle)
            t_rect = t_rot + self.ax.transData
            wheel.set_transform(t_rect)

            # if force > 0:
            #     wheel.set_color((0, force, 0, 1.0))
            # else:
            #     wheel.set_color((abs(force), 0, 0, 1.0))




        # update plot
        plt.pause(dt) # TODO: blit for better FPS



        

