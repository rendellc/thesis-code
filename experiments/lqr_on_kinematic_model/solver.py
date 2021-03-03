import numpy as np
from numpy.linalg import inv

from scipy.linalg import solve_continuous_are

import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

from blitmanager import BlitManager

# Live plots
fig, ax = plt.subplots(1,1)
etaline, = ax.plot([],[],animated=True)
etaarrow = ax.arrow(0,0,1,0,animated=True)
etadline, = ax.plot([],[],animated=True)
etadarrow = ax.arrow(0,0,1,0,animated=True)
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
ax.set_aspect("equal")
ax.grid(True)

bm = BlitManager(fig.canvas, [etaline, etaarrow, etadline, etadarrow])
plt.show(block=False)
plt.pause(0.1)

# Initial conditions
eta = np.array([0,5,0])
xw = np.array([0.1,0,0,0,0,0,0,0])

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

b = lambda xw: np.array([
    np.cos(xw[1])*xw[0],
    np.sin(xw[1])*xw[0],
    np.cos(xw[3])*xw[2],
    np.sin(xw[3])*xw[2],
    np.cos(xw[5])*xw[4],
    np.sin(xw[5])*xw[4],
    np.cos(xw[7])*xw[6],
    np.sin(xw[7])*xw[6]
])
def db(xw):
    # xw = [omega_fl, delta_fl, omega_rl, delta_rl, ...]
    dbxw = np.zeros((xw.size,xw.size))
    
    for i in range(0,xw.size,2):
        omega = xw[i+0]
        delta = xw[i+1]
        dbxw[i:i+2,i:i+2] = [
            [np.cos(delta), -np.sin(delta)*omega],
            [np.sin(delta), np.cos(delta)*omega]
        ]

    return dbxw



def eta_desired(t):
    eta = np.array([
        0 + 5*np.sin(0.02*t),
        0 + 5*np.cos(0.02*t),
        -0.02*t,
    ])

    return eta
    

T_delta = 0.5
T_omega = 0.5
Tinv = np.kron(np.eye(4), np.diag([1/T_omega,1/T_delta]))
x = np.empty((eta.size+xw.size,))

# LQR parameters
Qx,Qy,Qpsi = 1,1,1
Romega, Rdelta = 1, 1
Q = np.diag([Qx,Qy,Qpsi,0,0,0,0,0,0,0,0])
R = np.kron(np.eye(4), np.diag([Romega,Rdelta]))
Rinv = np.kron(np.eye(4), np.diag([1/Romega,1/Rdelta]))

# LTV system approximation
# Non-linear parts updated in loop
At = np.zeros((11,11))
Bt = np.zeros((x.size,8))
At[3:,3:] = -Tinv
Bt[3:,:] = Tinv
C = np.zeros((3,11))
C[0,0] = 1
C[1,1] = 1
C[2,2] = 1


t, dt, tstop = 0, 0.1, 1000
while t < tstop:
    # State 
    x[:eta.size] = eta
    x[eta.size:] = xw
    psi = eta[2]

    # Reference signal
    etad = eta_desired(t)

    
    # update date plot data
    etaline.set_xdata(np.append(etaline.get_xdata(), eta[0]))
    etaline.set_ydata(np.append(etaline.get_ydata(), eta[1]))
    etaarrow.set_transform(
            Affine2D().rotate(eta[2]).translate(eta[0],eta[1]) + ax.transData
    )
    etadline.set_xdata(np.append(etadline.get_xdata(), etad[0]))
    etadline.set_ydata(np.append(etadline.get_ydata(), etad[1]))
    etadarrow.set_transform(
            Affine2D().rotate(etad[2]).translate(etad[0],etad[1]) + ax.transData
    )


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
    bxw = b(xw)
    dbxw = db(xw)
    At[:3,2] = dJdpsi.dot(A).dot(bxw)
    At[:3,3:] = J.dot(A).dot(dbxw)

    ## Compute control law
    Pinf = solve_continuous_are(At,Bt,Q,R)
    G1 = -Rinv @ Bt.T @ Pinf
    Ac = At + Bt @ G1
    G2 = -Rinv @ Bt.T @ inv(Ac).T @ C.T @ Q[:3,:3]
    
    u = G1 @ x + G2 @ etad
    
    etadot = J.dot(A).dot(b(xw))
    xwdot = Tinv.dot(u - xw)
    
    # integrate
    eta = eta + dt*etadot
    xw = xw + dt*xwdot
    t += dt
    
    # update plot
    bm.update()
    

print(eta, etad)