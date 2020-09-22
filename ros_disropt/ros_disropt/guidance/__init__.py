from recordclass import recordclass, RecordClass # this class provides "mutable" namedtuples

class OptimizationSettings(RecordClass):
    max_iterations: int
    epsilon: float
    runavg: bool

class RobotData(RecordClass):
    speed: float

from .guidance import Guidance