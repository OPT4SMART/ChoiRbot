from rclpy.action import ActionClient
from typing import Callable
from ros_disropt_interfaces.action import PositionAction
from ros_disropt_interfaces.msg import PositionTask
from ros_disropt_interfaces.srv import PositionTaskService
from .. import Guidance
import numpy as np

class TaskExecutor:

    service = None
    guidance = None
    node_callback = None
    _current_task = None
    
    def initialize(self, guidance: Guidance):
        self.guidance = guidance
    
    def execute_async(self, task, callback: Callable):
        raise NotImplementedError

class PositionTaskExecutor(TaskExecutor):

    service = PositionTaskService
    _position_client = None
    _send_goal_future = None
    _get_result_future = None
    _gc_task_end = None

    def initialize(self, guidance: Guidance):
        super(PositionTaskExecutor, self).initialize(guidance)

        # create guard condition to be called at end of task execution
        self._gc_task_end = guidance.create_guard_condition(self._task_ended_callback)

        # create position action client
        self._position_client = ActionClient(
            guidance,
            PositionAction,
            'positionaction')

        # wait for action server
        self._position_client.wait_for_server()
    
    def execute_async(self, task: PositionTask, callback: Callable):
        if self._current_task is not None:
            raise RuntimeWarning("Skipping task: there is already one executing")
        
        self.node_callback = callback
        self._current_task = task

        # prepare goal object
        goal_msg = PositionAction.Goal()
        goal_msg.goal_position = task.coordinates

        # send goal and prepare callback
        self._send_goal_future = self._position_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._goal_ended_callback)
    
    def _goal_ended_callback(self, future):
        # extract result
        result = future.result().result

        # log to console new position
        final_position = np.array(result.final_position)
        self.guidance.get_logger().info('Task: moved to position: {}'.format(final_position))

        # trigger end of task
        self._gc_task_end.trigger()
    
    def _task_ended_callback(self):
        self._current_task = None

        if self.node_callback:
            self.node_callback()