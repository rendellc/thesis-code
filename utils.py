import numpy as np
from numpy import cos, sin

def skew(vector):
    vx, vy, vz = vector
    return np.array([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy,vx,0]])

def rotz(angle):
    return np.array([
            [cos(angle), -sin(angle), 0],
            [sin(angle),  cos(angle), 0],
            [0, 0, 1]])

def chunks(array, num_chunks):
    assert len(array) % num_chunks == 0, "num_chunks does not evenly split array"

    chunk_size = len(array) // num_chunks
    for i in range(0,len(array),chunk_size):
        yield array[i:i+chunk_size]

def sign(x: np.ndarray):
    """
    Compute sign with zero defined as positive sign.
    """
    sign = np.ones_like(x)
    sign[x<0] = -1
    return sign

def weight(mass):
    return 9.81*mass


def interleave(lists):
    if not lists:
        return []

    results = [val for tup in zip(*lists) for val in tup]
    assert len(lists)*len(lists[0]) == len(results), "All lists in lists must have same length"
    return results


