import numpy as np
from scipy.signal import lti,step

import matplotlib.pyplot as plt

kp,ki,kd = 22.2, 2.22, 35
J = 15.4

tf = lti([kd,kp,ki],[J,kd,kp,ki])

t,s = step(tf, T=np.linspace(0,10,100))

plt.plot(t,s)

plt.show(block=False)
