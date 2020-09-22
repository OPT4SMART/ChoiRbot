from abc import ABC, abstractmethod
from typing import List, TypeVar, Generic

ArrayType = TypeVar('ArrayType')
MsgType = TypeVar('MsgType')

class TaskManager(Generic[MsgType, ArrayType], ABC):

    def __init__(self):
        self.task_queue = []

    @abstractmethod
    def filter_tasks(self, task_list: ArrayType) -> ArrayType:
        pass
    
    @abstractmethod
    def has_tasks(self) -> bool:
        pass
    
    @abstractmethod
    def get_task(self) -> MsgType:
        pass
    
    @abstractmethod
    def update_tasks(self, task_list: List[MsgType]):
        pass

class DynamicTaskManager(TaskManager[MsgType, ArrayType]):    
    
    def filter_tasks(self, task_list: ArrayType) -> ArrayType:
        # task_list is the list of tasks to be executed received by central server
        # TODO delete task that the agent is currently executing (?) or perhaps also the next one
        return task_list
    
    def has_tasks(self):
        return bool(self.task_queue)

    def get_task(self) -> MsgType:
        if self.task_queue:
            return self.task_queue.pop(0)
        
        return None
    
    def get_all_tasks(self):
        return self.task_queue.copy()
    
    def update_tasks(self, task_list: List[MsgType]):
        # TODO remove already completed tasks
        self.task_queue = task_list
    
    def empty_tasks(self):
        # remove all tasks
        self.task_queue = []