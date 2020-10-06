from recordclass import recordclass, RecordClass # this class provides "mutable" namedtuples

class RobotData(RecordClass):
    speed: float

from .guidance import Guidance, OptimizationGuidance