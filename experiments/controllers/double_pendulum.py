import numpy as np
from scipy.linalg import inv
from dynamical_system import DynamicalSystem


class DoublePendulum(DynamicalSystem):
    def __init__(self, parameters):
        self.g = parameters["gravity"]
        self.m1 = parameters["link1_mass"]
        self.m2 = parameters["link2_mass"]
        self.L1 = parameters["link1_length"]
        self.L2 = parameters["link2_length"]
        self.I = self.m*self.L**2
        
        self._statesdot = np.empty((2,))
        self._Ak = np.eye(2)
        self._Bk = np.zeros((2,1))


    def fc(self, states, inputs):
        self._statesdot[0] = states[1]
        self._statesdot[1] = np.sin(states[0])*self.m*self.g*self.L/self.I + inputs*self.L/self.I
        return self._statesdot

    def A(self, state):
        return np.array([
            [0, 1],
            [np.cos(state[0])*self.m*self.L/self.I, 0]
        ])
    
    def B(self, state):
        return np.array([
            0,
            self.L/self.I
        ])
    
    def linearize(self, state, input):
        return self.A(state), self.B(state)
        

    def Ak(self, states, input, stepsize):
        h = stepsize
        A = self.A(state)
        I = np.eye(len(state))
        self._Ak = (I + h*A) # @ inv(I - h*A)

        return self._Ak

    def Bk(self, states, inputs, stepsize):
        self._Bk[1,0] = stepsize*self.L/self.I
        return self._Bk


DEFAULT_PARAMETERS = dict(
    rod_length=3,
    ball_mass=1,
    gravity=9.81
)