import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import math






N = 4
d = 2

A = np.zeros(((N-1)*d,N*d))
for i in range((N-1)*d):
    A[i,i] = -1
    A[i,i+2] = 1


p0 = np.array([1,1])
v0 = np.array([5.0, -1.0])
pn = np.array([5,-1])

x0 = np.zeros((N,d))
x0[0] = p0 # start point
x0[1] = p0 + v0
x0[~0] = pn # end point


fun = lambda x: np.sum((A@x)**2)
cons = (
        {"type": "eq", "fun": lambda x: x[0] - p0[0]},
        {"type": "eq", "fun": lambda x: x[1] - p0[1]},
        {"type": "eq", "fun": lambda x: x[2] - p0[0]-v0[0]},
        {"type": "eq", "fun": lambda x: x[3] - p0[1]-v0[1]},
        {"type": "eq", "fun": lambda x: x[~1] - pn[0]},
        {"type": "eq", "fun": lambda x: x[~0] - pn[1]},
        )
res = minimize(fun, x0, method="SLSQP", constraints=cons)

x = res.x.reshape((N,d))

def nCr(n,r):
    fac = math.factorial
    return fac(n)/(fac(r)*fac(n-r))

def generate_bezier_curve(points):
    degree = points.shape[0] - 1

    bs = [nCr(degree, i)*p for i,p in zip(range(degree+1),points)]

    def curve(t):
        return sum([b*(1-t)**(degree-i)*t**i for i,b in zip(range(degree+1), bs)])

    return curve



ctrl_points = x
res_curve = generate_bezier_curve(ctrl_points)

ts = np.linspace(0,1,100)
ps = []
for t in ts:
    ps.append(res_curve(t))

ps = np.array(ps)
print(ps)


plt.plot(ps[:,0], ps[:,1])

plt.xlim([-10,10])
plt.ylim([-10,10])

plt.show(block=False)



