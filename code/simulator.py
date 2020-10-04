
from typing_extensions import Protocol, runtime
from typing import List

from dynamicmodels import DynamicModel


class DynSysSim:
    @staticmethod
    def step(model : DynamicModel, inputs : List[float], step_size : float) -> DynamicModel:
        """
        Perform one step of simulation method. Return new state
        """
        # Eulers method
        return DynSysSim.predict(model, inputs, step_size)



    @staticmethod
    def predict(model : DynamicModel, inputs : List[float], step_size : float) -> DynamicModel:
        """
        Use Euler method to predict state after time step_size.
        """
        x = model.states()
        dxdt = model.derivatives(inputs)
        xpredict = x + step_size*dxdt
        modelpredict = model.from_states(xpredict)
        return modelpredict


