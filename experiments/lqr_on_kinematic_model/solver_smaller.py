import numpy as np
from numpy.linalg import inv

from scipy.linalg import solve_continuous_are

import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

from blitmanager import BlitManager

# Live plots
fig, ax = plt.subplots(1,3)
etaline, = ax[0].plot([],[],animated=True)
etaarrow = ax[0].arrow(0,0,1,0,animated=True)
etadline, = ax[0].plot([],[],animated=True)
etadarrow = ax[0].arrow(0,0,1,0,animated=True)
ax[0].set_xlim(-10,10)
ax[0].set_ylim(-10,10)
ax[0].set_aspect("equal")
ax[0].grid(True)

poserrorline, = ax[1].plot([],[],animated=True)
yawerrorline, = ax[2].plot([],[],animated=True)
ax[2].set_ylim(0,np.pi)

bm = BlitManager(fig.canvas, [etaline, etaarrow, etadline, etadarrow, poserrorline, yawerrorline])
plt.show(block=False)
plt.pause(0.1)

# Initial conditions
eta = np.array([0,5,0])
eta_integral = np.zeros_like(eta)

# Setup kinematic model system
wheel_radius = 0.505
wheel_positions = np.array([
    [1,1],
    [-1,1],
    [-1,-1],
    [1,-1],
])
G = np.empty((8,3))
for ij in range(4):
    p = np.cross([0,0,1],wheel_positions[ij])[:2]
    G[2*ij:2*ij+2,:2] = np.eye(2)
    G[2*ij:2*ij+2,2] = p

A = wheel_radius * np.linalg.inv(G.T.dot(G)).dot(G.T)

b = lambda u: np.array([
    np.cos(u[1])*u[0],
    np.sin(u[1])*u[0],
    np.cos(u[3])*u[2],
    np.sin(u[3])*u[2],
    np.cos(u[5])*u[4],
    np.sin(u[5])*u[4],
    np.cos(u[7])*u[6],
    np.sin(u[7])*u[6]
])
def linearize_b(u):
    # u = [omega_fl, delta_fl, omega_rl, delta_rl, ...]
    Bt = np.zeros((u.size,u.size))
    
    for i in range(0,u.size,2):
        omega = u[i+0]
        delta = u[i+1]
        Bt[i:i+2,i:i+2] = [
            [np.cos(delta), -np.sin(delta)*omega],
            [np.sin(delta), np.cos(delta)*omega]
        ]

    return Bt

def ssa(ts):
    return np.arctan2(np.sin(ts), np.cos(ts))


def eta_desired(t):
    eta = np.array([
        0 + 5*np.sin(0.02*t),
        0 + 5*np.cos(0.02*t),
        -0.02*t
    ])

    return eta
    

T_delta = 0.5
T_omega = 0.5
Tinv = np.kron(np.eye(4), np.diag([1/T_omega,1/T_delta]))

# LQR parameters
Qx,Qy,Qpsi = 1,1,1
Romega, Rdelta = 1, 1
Q = np.diag([Qx,Qy,Qpsi]) 
R = np.kron(np.eye(4), np.diag([Romega,Rdelta]))
Rinv = np.kron(np.eye(4), np.diag([1/Romega,1/Rdelta]))

# LTV system approximation
# Non-linear parts updated in loop
x = np.hstack([eta]) #, eta_integral])
At = np.zeros((3,3))
#At[3:,:3] = np.eye(3)
Bt = np.zeros((3,8))
C = np.zeros((3,3))
C[:3,:3] = np.eye(3)

t, dt, tstop = 0, 0.01, 1000
u_previous = np.array([0,np.pi/10, 0,0, 0,0, 0,np.pi/10])
while t < tstop:
    # State 
    x[:3] = eta
    #x[3:] = eta_integral
    px, py = eta[:2]
    psi = eta[2]

    # Reference signal
    etad = eta_desired(t)
    
    # update date plot data
    etaline.set_xdata(np.append(etaline.get_xdata(), eta[0]))
    etaline.set_ydata(np.append(etaline.get_ydata(), eta[1]))
    etaarrow.set_transform(
            Affine2D().rotate(eta[2]).translate(eta[0],eta[1]) + ax[0].transData
    )
    etadline.set_xdata(np.append(etadline.get_xdata(), etad[0]))
    etadline.set_ydata(np.append(etadline.get_ydata(), etad[1]))
    etadarrow.set_transform(
            Affine2D().rotate(etad[2]).translate(etad[0],etad[1]) + ax[0].transData
    )
    poserror = np.linalg.norm(eta[:2] - etad[:2])
    poserrorline.set_xdata(np.append(poserrorline.get_xdata(), t))
    poserrorline.set_ydata(np.append(poserrorline.get_ydata(), poserror))
    ax[1].set_xlim(0,t+1)
    ax[1].set_ylim(0,1)
    yawerror = abs(ssa(eta[2] - etad[2]))
    yawerrorline.set_xdata(np.append(yawerrorline.get_xdata(), t))
    yawerrorline.set_ydata(np.append(yawerrorline.get_ydata(), yawerror))
    ax[2].set_xlim(0,t+1)


    J = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1],
    ])
    dJdpsi = np.array([
        [-np.sin(psi),-np.cos(psi),0],
        [np.cos(psi),-np.sin(psi),0],
        [0,0,0]
    ])
    
    # LQR control
    ## Update LTV system
    B_linear = linearize_b(u_previous)
    Bt[:3] = J @ A @ B_linear

    ## Compute control law
    Pinf = solve_continuous_are(At,Bt,Q,R)
    G1 = -Rinv @ Bt.T @ Pinf
    Ac = At + Bt @ G1
    G2 = -Rinv @ Bt.T @ inv(Ac).T @ C.T @ Q[:3,:3]
    
    u = G1 @ x + G2 @ etad
    
    
    # integrate
    etadot = J.dot(A).dot(b(u))
    eta = eta + dt*etadot
    t += dt
    u_previous = u
    
    # update plot
    bm.update()
    
print(eta, etad)