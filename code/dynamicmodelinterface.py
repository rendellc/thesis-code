from typing import List, TypeVar
from typing_extensions import Protocol, runtime

# A type containing the state of the simulateable object
S = TypeVar("S")

@runtime
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

