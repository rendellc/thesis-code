
from typing_extensions import Protocol, runtime
from typing import List

S = TypeVar("S")

class DynamicSystemSimulation:
    def step(self, state : S, inputs : List[float], step_size : float) -> S:
        """
        Perform one step of simulation method. Return new state
        """
        ...


    def predict(self, state : S, inputs : List[float], step_size : float) -> S:
        """
        Use Euler method to predict state after time step_size.
        """
        ...

