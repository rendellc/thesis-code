import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from matplotlib.transforms import Affine2D

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

        R = self.wheel_radius
        T = self.wheel_thickness
        self.wheels = []
        alphas = [0.3,0.5,0.7,0.9]
        colors = ["red","green","blue","yellow"]
        for i in range(len(self.corners)):
            rect = mpl.patches.Rectangle((-R,-T/2), 2*R, T, color=colors[i], ec="k", linewidth=1)
            self.ax.add_patch(rect)
            self.wheels.append(rect)

    def update(self, t, position, chassis_angle, wheel_angles, dt=1e-10):
        # force_total = np.sum(np.abs(wheel_forces))
        # force_ratios = wheel_forces/(force_total+1e-8)

        self.ax.set_title(f"time {t:.4f}")

        x, y = position[0], position[1]

        # transformations
        # t_chassis = Affine2D().rotate(chassis_angle).translate(x,y)
        t_rot = Affine2D().rotate(chassis_angle)
        t_trans = Affine2D().translate(x,y)
        # t_chassis = Affine2D().translate(x,y).rotate_around(x,y,chassis_angle)
        t_chassis = t_rot + t_trans
        self.chassis.set_transform(t_chassis + self.ax.transData)

        c, s = np.cos(chassis_angle), np.sin(chassis_angle)
        rot_chassis = np.array([
            [c, -s],
            [s, c]])

        for corner, wheel, wheel_angle in zip(self.corners, self.wheels, wheel_angles):
            xc, yc = rot_chassis @ np.array(corner)
            xw, yw = x+xc, y+yc
            t_wheel = Affine2D().translate(xc,yc).rotate_around(xw,yw,wheel_angle)

            wheel.set_transform(t_chassis + t_wheel + self.ax.transData)

class TimeSeries:
    def __init__(self, xs0, ys0, label, ax=None):
        self.xs = xs0
        self.ys = ys0

        if ax is None:
            fig, ax = plt.subplots(1,1)

        self.ax = ax


    def update(self, x, y):
        pass


