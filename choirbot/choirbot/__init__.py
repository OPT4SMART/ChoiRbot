from recordclass import recordclass, RecordClass # this class provides "mutable" namedtuples
import numpy as np

class Pose(RecordClass):
    position: np.ndarray
    orientation: np.ndarray
    velocity: np.ndarray
    angular: np.ndarray