from dataclasses import dataclass
import numpy as np

@dataclass
class PID:
    P: float
    I: float
    D: float

    integral: float = 0
    previous_error: float = None

    def __call__(self, dt, error, errorrate=None):
        self.integral += dt*error

        P_effect = self.P*error
        I_effect = self.I*self.integral

        D_effect = 0
        if errorrate is None and self.previous_error is not None:
            errorrate = (error - self.previous_error)/dt
            D_effect = self.D*errorrate

        self.previous_error = error

        return P_effect + I_effect + D_effect

    def reset(self):
        self.integral = 0


def ssa(angle):
    """
    angle: float or [float]
    return smallest signed equivalent angle(s)
    """
    return np.arctan2(np.sin(angle), np.cos(angle))

