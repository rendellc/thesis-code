

class Simulatable:
    """
    Interface for simulatable objects.

    TODO: would be better to encapsulate state in a 
    variable and then implement all simulation logic here.
    """
    def __init__(self):
        pass

    def compute_derivatives(self):
        """
        Compute derivatives and store them in object.
        """
        pass

    def get_simulatable_children(self):
        return []

    def step_forward(self, stepsize):
        """
        Use the computed derivatives to step simulation forward.
        """
        pass


def simulate_step_forward(model, stepsize):
    model.compute_derivatives()
    children = model.get_simulatable_children()
    for child in children:
        simulate_step_forward(child, stepsize)
    model.step_forward(stepsize)



