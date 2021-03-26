import numpy as np
from scipy.linalg import expm



class DynamicalSystem:
    def fc(self, state, inputs):
        """
        Continuous state space model: \dot{x} = f_c(x,u)
        """
        pass

    def fk(self, state, inputs, stepsize):
        """
        Discrete state space model: x_{k+1} = f_k(x_k, u_k)
        """
        # Runge-Kutta 4
        u = inputs
        x = state
        h = stepsize

        k1 = self.fc(x, u)
        k2 = self.fc(x + h/2*k1, u)
        k3 = self.fc(x + h/2*k2, u)
        k4 = self.fc(x + h*k3, u)
        
        return x + h/6*(k1 + 2*k2 + 2*k3 + k4)

    def linearize(self, state, inputs):
        """
        Return A,B for approximate continuous LTI system around operating point.
        """
        pass

    def discretize(self, state, inputs, stepsize):
        """
        Return Ak, Bk for approximate discrete LTI system around operating point.
        """
        A, B = self.linearize(state, inputs)
        nx = len(state)
        nu = len(inputs)
        F = np.zeros((nx+nu, nx+nu))
        F[:nx,:nx] = A * stepsize
        F[:nx,nx:] = (B * stepsize).reshape((nx,nu))
        V = expm(F)
        
        Ak = V[:nx,:nx]
        Bk = V[:nx,nx:]
        return Ak, Bk



    
        
    def Ak(self,state,inputs):
        """
        Gradient of fk with respect to states: D_x f_k
        """
        pass

    def Bk(self,state,inputs):
        """
        Gradient of fk with respect to inputs: D_u f_k
        """
        pass
        