import numpy as np
#from lqr import InfiniteHorizonLQR
from ilqr import IterativeLQR
from lqr import InfiniteHorizonLQR
from inverted_pendulum import InvertedPendulum, DEFAULT_PARAMETERS

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.transforms import Affine2D






dynsys = InvertedPendulum(DEFAULT_PARAMETERS)
x = np.array([1.8, 0])

t, dt, tstop = 0, 0.05, 20

# Controller setup
N = 20
u_trajectory = [np.array([0])]*N
#controller = IterativeLQR(dynsys, [0, 0], [1, 1], [1e-8], x, u_trajectory, dt)
controller = InfiniteHorizonLQR(dynsys, [1, 1], [1e-8])
    


fig, ax = plt.subplots()
ax.set_xlim(-dynsys.L*1.5, dynsys.L*1.5)
ax.set_ylim(-dynsys.L*1.5, dynsys.L*1.5)
#ax.grid(True)
ax.set_aspect("equal")
rodwidth = dynsys.L/100
pos = dynsys.L*np.array([np.sin(x[0]), np.cos(x[0])])
rod =  Rectangle((-rodwidth/2,0), rodwidth, dynsys.L)
ball = Circle((0, dynsys.L), radius=0.2)

ax.add_patch(rod)
ax.add_patch(ball)
plt.show(block=False)


while t < tstop:
    x_target = np.array([
        -1 if t < 5 else 1,
        0
    ])
    u = controller.update(x, x_target, dt)
    # u = 0
    x = dynsys.fk(x, u, dt)
    t += dt
    
    # Animate
    rotTrans = Affine2D().rotate(x[0])
    rod.set_transform(
        rotTrans + ax.transData
    )
    ball.set_transform(
        rotTrans + ax.transData
    )
    print(u, x[0] - x_target[0])
    plt.pause(0.0001)



