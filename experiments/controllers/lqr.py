
import numpy as np
from dynamical_system import DynamicalSystem

from scipy.linalg import solve_discrete_are
from scipy.linalg import inv

class InfiniteHorizonLQR:
    def __init__(self,
        dynsys: DynamicalSystem,
        cost_states: np.ndarray,
        cost_inputs: np.ndarray
    ):
        if np.array(cost_states).ndim == 1:
            self.Q = np.diag(cost_states)
        else:
            self.Q = cost_states

        if np.array(cost_inputs).ndim == 1:
            self.R = np.diag(cost_inputs)
        else:
            self.R = cost_inputs
        
        self.dynsys = dynsys
        
        # Dummy values, will be overriden in update unless they happen to be correct
        self.Ak_prev = 1
        self.Bk_prev = 1
        self.P = solve_discrete_are(self.Ak_prev, self.Bk_prev, 1, 1)

        

    def update(self, state, target, stepsize):
        # TODO: figure out inputs here
        inputs = [0]
        Ak, Bk = self.dynsys.discretize(state, inputs, stepsize)

        if not (np.array_equal(Ak,self.Ak_prev) and np.array_equal(Bk, self.Bk_prev)):
            # Solve DARE
            P = solve_discrete_are(Ak, Bk, self.Q, self.R)
            # shorter names
            R = self.R
            # feedback matrix
            self.K = inv(R + Bk.T @ P @ Bk) @ (Bk.T @ P @ Ak )

        self.Ak_prev = Ak
        self.Bk_prev = Ak
        
        return self.K @ (target - state)
        
        