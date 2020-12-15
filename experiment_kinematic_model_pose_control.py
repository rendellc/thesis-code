#!/usr/bin/env python

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import solve_continuous_are

#plt.style.use(["science","no-latex"])
plt.style.use(["science","ieee","no-latex"])
figsize = (3.5,2.2)
np.set_printoptions(precision=4)

from simulator import VehicleSim
from simulator.common import PID
from simulator.common import ssa
from simulator.rotations import rot_z, rot_z_der
import liveplot
from blitmanager import BlitManager


import tqdm
import time

import params
vp = params.VEHICLE_PARAMS
sp = params.SIM_PARAMS
sp["do3Dview"] = False
wheel_radius = vp["wheel_radius"]
sim = VehicleSim(vp, sp)
wheel_arms = sim.getWheelPositions()


A = np.zeros((8,3))
zu = np.array([0,0,1])
A[::2,0] = 1
A[1::2,1] = 1
A[:,2] = np.cross(zu,wheel_arms)[:,:2].flatten()
Ainv = np.linalg.inv(A.T.dot(A)).dot(A.T)

eta = np.array([0,0,0])
eta_J = np.eye(3) # transformation from body to inertial frame
eta_Jinv = np.eye(3) # transformation from inertial to body frame
nu = np.array([0,0,0])

x = np.hstack([eta,nu])

def eta_ref_generator():
    eta_refs = np.array([
        [5,-2, 1.0],
        [5,-2, 0.0],
        [-5,-2, 0.0],
        [5,2, -1.0]
    ])
    i = 0
    while True:
        yield eta_refs[i]
        i = (i + 1) % len(eta_refs)

ref_gen = eta_ref_generator()

B = Ainv
Kp = np.zeros((8,3))
Kd = np.zeros((8,3))

# xdot = Fx + Gu
F = np.zeros((6,6))
F[:3,3:] = np.eye(3)
G = wheel_radius*np.vstack([np.zeros((3,8)),B])


Nx, Nu = 6, 8
C = np.zeros((Nx,Nx*Nu))
Fp = np.eye(Nx)
for i in range(Nx):
    C[:,i*Nu:(i+1)*Nu] = Fp @ G
    Fp = F @ Fp
C = np.array(C)

assert Nx == np.linalg.matrix_rank(C), "System is not controllable"

# Optimal full state feedback control using LQR
# 13.165 and 13.166 in Fossen
# https://onlinelibrary.wiley.com/doi/pdf/10.1002/9781119994138
Q = np.diag([1,1,1,1,1,1]) # state penalty
R = np.eye(8) # input penalty
# Solve Ricatti equations
# Pinf F + F.T Pinf - Pinf G R^-1 G.T Pinf + Q = 0
Pinf = solve_continuous_are(F, G, Q, R)
K_LQ = np.linalg.inv(R) @ G.T @ Pinf

fps = 30
tstart,dt,tstop = 0,1/fps,300
steps = int((tstop - tstart)/dt)

t = np.linspace(tstart,tstop,steps)
x = np.zeros((steps,6))
eta_ref = np.zeros((steps,3))
xdot = np.zeros_like(x)
error = np.zeros_like(x)
u = np.zeros((steps,8))

eta_ref[0] = next(ref_gen)

for step in tqdm.tqdm(range(steps-1)):
    # get current state
    eta, nu = x[step,:3], x[step,3:]
    eta_J[:2,:2] = rot_z(eta[2])[:2,:2]
    eta_Jinv[:2,:2] = rot_z(-eta[2])[:2,:2]


    # compute control inputs
    # TODO: ssa on angle
    eta_body_err = eta_Jinv @ (eta - eta_ref[step])
    error[step] = np.hstack([eta_body_err, nu])

    u[step] = -K_LQ @ error[step]
    b = wheel_radius*u[step]

    # compute derivatives
    nudot = Ainv @ b
    etadot = eta_J @ nu
    xdot[step] = np.hstack([etadot, nudot])

    # update state
    x[step+1] = x[step] + dt*xdot[step]

    # change reference for next step?
    if np.linalg.norm(error[step]) < 1e-3:
        eta_ref[step+1] = next(ref_gen)
    else:
        eta_ref[step+1] = eta_ref[step]




# plotting
pos = x[:,:2]
yaw = x[:,2]
yawrate = x[:,5]

u = u.reshape(steps,4,2)
omega = np.linalg.norm(u,axis=2)
steer = np.arctan2(u[:,:,1],u[:,:,0])

fig, ax = plt.subplots(1,1,num="Trajectory")
ax.plot(pos[:,0], pos[:,1], label="pos")
ax.set_aspect("equal")

fig, ax = plt.subplots(1,1,num="Yaw")
ax.plot(t, yaw, label=r"$\psi$")
ax.set_xlim(t[0],t[~0])

fig, ax = plt.subplots(1,1,num="Yaw Rate")
ax.plot(t, yawrate, label=r"$\dot{\psi}$")
ax.set_xlim(t[0],t[~0])

fig, ax = plt.subplots(1,1,num="Omega")
ax.plot(t, omega[:,0], label="FL")
ax.plot(t, omega[:,1], label="RL")
ax.plot(t, omega[:,2], label="RR")
ax.plot(t, omega[:,3], label="FR")
ax.set_xlim(t[0],t[~0])

# animation
fig, ax = plt.subplots()

ln, = ax.plot([],[])
posearrow = ax.arrow(0,0,1,0,color="r")
refarrow = ax.arrow(0,0,1,0,color="g")
#refarrow = ax.arrow(eta_ref[0,0],eta_ref[0,1],np.cos(eta_ref[2]),np.sin(eta_ref[2]),color="g")

def arrowtransform(pos,yaw):
    t_translate = mpl.transforms.Affine2D().translate(pos[0],pos[1])
    t_rotate = mpl.transforms.Affine2D().rotate(yaw)
    return t_rotate + t_translate + ax.transData


def init():
    posearrow.set_transform(ax.transData)
    ax.set_xlim(-10,10)
    ax.set_ylim(-5,5)
    ax.set_aspect("equal")
    ax.grid(True)
    return ln, posearrow, refarrow
def update(frame):
    posearrow.set_transform(arrowtransform(pos[frame],yaw[frame]))
    refarrow.set_transform(arrowtransform(eta_ref[frame,:2],eta_ref[frame,2]))
    ln.set_data(pos[:frame,0],pos[:frame,1])
    return ln, posearrow, refarrow

ani = FuncAnimation(fig, update, 
        frames=range(steps-1), interval=dt,
        init_func=init, blit=True)

fig.show()
#plt.show(block=False)

