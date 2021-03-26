
import numpy as np
from dynamical_system import DynamicalSystem

from scipy.linalg import solve_discrete_are
from scipy.linalg import inv

from typing import List

class IterativeLQR:
    def __init__(self,
        dynsys: DynamicalSystem,
        cost_states: np.ndarray,
        cost_states_final: np.ndarray,
        cost_inputs: np.ndarray,
        initial_state: np.ndarray,
        input_sequence: np.ndarray,
        stepsize: float
    ):
        if np.array(cost_states).ndim == 1:
            self.Q = np.diag(cost_states)
        else:
            self.Q = cost_states

        if np.array(cost_states_final).ndim == 1:
            self.Qf = np.diag(cost_states_final)
        else:
            self.Qf = cost_states_final

        if np.array(cost_inputs).ndim == 1:
            self.R = np.diag(cost_inputs)
            self.Rinv = np.diag(1/np.array(cost_inputs))
        else:
            self.R = cost_inputs
            self.Rinv = inv(R)
        
        self.dynsys = dynsys

        self.input_sequence = input_sequence
        self.N = len(input_sequence)

    def update(self, 
        state: np.ndarray,
        state_target: np.ndarray,
        stepsize: float
    ):
        """
        state: [x_0, x_1, ..., x_{N-1}, x_N]
        target: x_N^*
        stepsize: h
        """
        # Variable names closer to litterature
        # https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf
        N = self.N
        u = self.input_sequence
        x_target = state_target
        
        # dx_1 = Aks[0]*dx_0 + Bks[0]*du_0
        # ...
        # dx_N = Aks[N-1]*dx_{N-1} + Bks[N-1]*du_{N-1}
        x = [state]*(N+1)
        A = [None]*N
        B = [None]*N
        S = [None]*(N+1)
        v = [None]*(N+1)
        for k in range(N):
            x[k+1] = self.dynsys.fk(x[k], u[k], stepsize)
            A[k], B[k] = self.dynsys.discretize(x[k], u[k], stepsize)
            # A[k] = self.dynsys.Ak(x[k], u[k], stepsize)
            # B[k] = self.dynsys.Bk(x[k], u[k], stepsize)
            
        # Backward pass to obtain S[k], v[k]
        K = [None]*len(u)
        Kv = [None]*len(u)
        Ku = [None]*len(u)
        S[N] = self.Qf
        v[N] = self.Qf @ (x[N] - x_target)
        I = np.eye(len(x[0]))
        for k in range(N-1,-1,-1):
            S[k] = A[k].T @ S[k+1] @ \
                (I - B[k] @ inv(B[k].T @ S[k+1] @ B[k] + self.R) @ B[k].T @ S[k+1]) @ A[k] \
                + self.Q
            
            V = inv(B[k].T @ S[k+1] @ B[k] + self.R)
            K[k] = V @ B[k].T @ S[k+1] @ A[k]
            Kv[k] = V @ B[k].T
            Ku[k] = V @ self.R
            
            v[k] = (A[k] - B[k] @ K[k]).T @ v[k+1] - K[k].T @ self.R @ u[k] + self.Q @ x[k]
            

        # Forward pass to obtain dx[k], du[k] and update optimal trajectories
        dx = [None]*len(x)
        dx[0] = np.zeros_like(x[0]) 
        du = [None]*len(u)
        for k in range(N):
            du[k] = -K[k] @ dx[k] - Kv[k] @ v[k+1] - Ku[k] @ u[k]
            dx[k+1] = A[k] @ dx[k] + B[k] @ du[k]

            u[k] = u[k] + du[k]
            x[k+1] = x[k+1] + dx[k+1]
        

        # Shift state/input sequences forward to set up for next
        u.append(
            u[~0]
        )
        x.append(
            self.dynsys.fk(x[~0], u[~0], stepsize)
        )
        # Control action to take current timestep
        u0 = u[0]
        x = x[1:]
        u = u[1:]

        self.state_sequence = x
        self.input_sequence = u
        
        return u0
        