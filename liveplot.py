import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from matplotlib.transforms import Affine2D

"""
from models.vehicle import VehicleModel
class VehicleAnimation:
    def __init__(self, v: VehicleModel, size=30, ax=None):
        self.width = v.body.width
        self.length = v.body.length
        self.wheel_thickness = v.wheels[0].width
        self.wheel_radius = v.wheels[0].radius

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
        x,y = v.body.pos_in[0], v.body.pos_in[1]
        B = self.width
        L = self.length
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
        self.chassis = mpl.patches.Polygon(xy, ec="k", linewidth=0.5)

        self.ax.add_patch(self.chassis)

        # draw wheels
        R = self.wheel_radius
        T = self.wheel_thickness
        colors = ["red","green","blue","yellow"]
        self.wheels = []
        for wheel,col in zip(v.wheels, colors):
            rect = mpl.patches.Rectangle((-R,-T/2), 2*R, T, color=col, ec="k", linewidth=0.5)
            self.ax.add_patch(rect)
            self.wheels.append(rect)

        self.corners = [
                (L/2, B/2),  # front left
                (-L/2, B/2), # rear left
                (-L/2, -B/2),  # rear right
                (L/2, -B/2)]   # front right

        #R = self.wheel_radius
        #T = self.wheel_thickness
        #self.wheels = []
        #alphas = [0.3,0.5,0.7,0.9]
        #colors = ["red","green","blue","yellow"]
        #for i in range(len(self.corners)):
        #    rect = mpl.patches.Rectangle((-R,-T/2), 2*R, T, color=colors[i], ec="k", linewidth=0.5)
        #    self.ax.add_patch(rect)
        #    self.wheels.append(rect)

    def update(self, t, v: VehicleModel, dt=1e-10):
        self.ax.set_title(f"time {t:.4f}")

        x, y = v.body.pos_in[0:2]

        # recompute axis limits
        growfactor = 1.1
        left, right = self.ax.get_xlim()
        bottom, top = self.ax.get_ylim()
        if min(x - self.width, x - self.width) < left:
            left = growfactor*left
        if min(x + self.width, x + self.width) > right:
            right = growfactor*right
        if min(y - self.width, y - self.width) < bottom:
            bottom = growfactor*bottom
        if min(y + self.width, y + self.width) > top:
            top = growfactor*top
        self.ax.set_xlim(left, right)
        self.ax.set_ylim(bottom, top)
        self.ax.set_aspect("equal")

        # transformations
        # t_chassis = Affine2D().rotate(v.body.yaw).translate(x,y)
        t_rot = Affine2D().rotate(v.body.yaw)
        t_trans = Affine2D().translate(x,y)
        t_chassis = t_rot + t_trans
        self.chassis.set_transform(t_chassis + self.ax.transData)

        c, s = np.cos(v.body.yaw), np.sin(v.body.yaw)
        rot_chassis = np.array([
            [c, -s],
            [s, c]])

        for w, patch in zip(v.wheels, self.wheels):
            xc, yc, zc = v.body.rot_body_to_in @ w.pos_b
            xw, yw = x+xc, y+yc
            t_wheel = Affine2D().translate(xc,yc).rotate_around(xw, yw, w.steer_angle)

            patch.set_transform(t_chassis + t_wheel + self.ax.transData)
"""


class Plot:
    def __init__(self, xs0, ys0, ax=None, **kwargs):
        self.xs = xs0
        self.ys = ys0
        self.rescale = kwargs.pop("rescale", True)

        if ax is None:
            fig, ax = plt.subplots(1,1)

        self.ax = ax
        self.line, = self.ax.plot(self.xs, self.ys, **kwargs)


    def update(self, x, y):
        self.xs.append(x)
        self.ys.append(y)
        self.line.set_data(self.xs, self.ys)

        if self.rescale:
            self.ax.relim()
            self.ax.autoscale_view()


class Marker:
    def __init__(self, x0, y0, ax=None, **kwargs):
        self.rescale = kwargs.pop("rescale", True)
        if ax is None:
            fig, ax = plt.subplots(1,1)

        self.ax = ax
        self.scat = self.ax.scatter(0,0, **kwargs)

        self.update(x0,y0)

    def update(self, x,y):
        self.scat.set_offsets((x,y))




