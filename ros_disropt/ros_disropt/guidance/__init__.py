from recordclass import recordclass, RecordClass # this class provides "mutable" namedtuples

class OptimizationSettings(RecordClass):
    max_iterations: int
    epsilon: float
    runavg: bool

class RobotData(RecordClass):
    capacity: float
    speed: float
    current_load: float

from .guidance import Guidance, TaskGuidance
from .task_executor import TaskExecutor, PositionTaskExecutor
from .task_manager import TaskManager, DynamicTaskManager
from .task_table import TaskTable, PositionTaskTable, PDPositionTaskTable