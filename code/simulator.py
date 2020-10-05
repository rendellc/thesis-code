
from typing import List, TypeVar
from typing_extensions import Protocol, runtime
from typing import List

S = TypeVar("S")

class Simulatable(Protocol[S]):
    def states(self) -> List[float]:
        """
        Get dynamic state variables.
        """
        ...

    def derivatives(self, inputs: List[float]) -> List[float]:
        """
        Get the derivative of the state variables.
        """
        ...

    def from_states(self, states: List[float]) -> S:
        """
        Create a model with same parameters, but with state variables
        coming from states array.
        """
        ...

class Solver:
    def step(self, model: Simulatable, inputs: List[float], step_size: float) -> Simulatable:
        ...


class ImprovedEuler(Solver):
    def step(self, model: Simulatable, inputs: List[float], step_size: float) -> Simulatable:
        """
        Perform one step of simulation method. Return new state
        """
        x1 = model.states()
        k1 = model.derivatives(inputs)

        x2 = x1 + step_size*k1
        k2 = model.from_states(x2).derivatives(inputs)

        xnew = x1 + step_size/2 * (k1 + k2)
        modelnew = model.from_states(xnew)
        return modelnew


class ForwardEuler(Solver):
    def step(self, model: Simulatable, inputs: List[float], step_size: float) -> Simulatable:
        x = model.states()
        dxdt = model.derivatives(inputs)
        xnew = x + step_size*dxdt
        modelnew = model.from_states(xnew)

        return modelnew

class RK4(Solver):
    def step(self, model: Simulatable, inputs: List[float], step_size: float) -> Simulatable:

        x1 = model.states()
        k1 = model.derivatives(inputs)

        x2 = x1 + step_size/2*k1
        k2 = model.from_states(x2).derivatives(inputs)

        x3 = x1 + step_size/2*k2
        k3 = model.from_states(x3).derivatives(inputs)

        x4 = x1 + step_size*k3
        k4 = model.from_states(x4).derivatives(inputs)

        xnew = x1 + 1/6*step_size*(k1 + 2*k2 + 2*k3 + k4)
        modelnew = model.from_states(xnew)

        return modelnew





