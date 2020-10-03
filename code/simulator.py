
from typing_extensions import Protocol, runtime
from typing import List



class DynamicSystemSimulation:
    def step(self, model : DynamicModel, inputs : List[float], step_size : float) -> S:
        """
        Perform one step of simulation method. Return new state
        """
        ...


    def predict(self, model : DynamicModel, inputs : List[float], step_size : float) -> S:
        """
        Use Euler method to predict state after time step_size.
        """
        ...

