from dataclasses import dataclass
from typing import Callable
from numpy import ndarray


@dataclass
class EulersMethod:
    f: Callable[[ndarray, float], ndarray]

    def step(self, x, t, h):
        return x + h*self.f(x,t)


@dataclass
class ImprovedEulersMethod:
    f: Callable[[ndarray, float], ndarray]

    def step(self,x,t,h):
        k1 = self.f(x,t)
        k2 = self.f(x+h*k1, t+h)

        return x + h/2*(k1 + k2)

@dataclass
class ModifiedEulersMethod:
    f: Callable[[ndarray, float], ndarray]

    def step(self,x,t,h):
        k1 = self.f(x,t)
        k2 = self.f(x+h/2*k1, t+h/2)

        return x + h*k2


if __name__=="__main__":
    def f(x,t): return -x**3

    em = EulersMethod(f)

    xs = [1]
    t, dt, tstop = 0, 0.01, 10
    while t < tstop:
        x = xs[~0]
        x = em.step(x,t,dt)
        xs.append(x)
        t += dt

    print(xs)



