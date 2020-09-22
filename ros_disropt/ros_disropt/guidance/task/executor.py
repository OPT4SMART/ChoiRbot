from rclpy.action import ActionClient
from rclpy.node import Node
from typing import Callable
from ros_disropt_interfaces.action import PositionAction
from ros_disropt_interfaces.msg import PositionTask
from ros_disropt_interfaces.srv import PositionTaskService
from .. import RobotData
import numpy as np

class TaskExecutor:

    service = None

    def __init__(self, agent_id: int, node: Node):
        super(TaskExecutor, self).__init__()

        self.agent_id = agent_id
        self.node = node
        self.node_callback = None
        self._current_task = None
    
    def execute_async(self, task, callback: Callable):
        raise NotImplementedError
    
    def wait_for_services(self):
        raise NotImplementedError

class PositionTaskExecutor(TaskExecutor):

    service = PositionTaskService

    def __init__(self, agent_id: int, node: Node, data: RobotData):
        super(PositionTaskExecutor, self).__init__(agent_id, node)

        self.data = data

        # path planner action client
        self.planner_client = ActionClient(
            node,
            PositionAction,
            'positiontask_{}'.format(agent_id))

        self._send_goal_future = None
        self._get_result_future = None

        # guard condition: end of task execution
        self._gc_task_end = node.create_guard_condition(self._task_ended_callback)
    
    def wait_for_services(self):
        self.planner_client.wait_for_server()
    
    def execute_async(self, task: PositionTask, callback: Callable):
        if self._current_task is not None:
            raise RuntimeWarning("Skipping task: there is already one executing")
        
        self.node_callback = callback
        self._current_task = task

        # prepare goal object
        goal_msg = PositionAction.Goal()
        goal_msg.goal_position = task.coordinates

        # send goal and prepare callback
        self._send_goal_future = self.planner_client.send_goal_async(goal_msg)
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
        self.node.get_logger().info('Task: moved to position: {}'.format(final_position))

        # trigger end of task
        self._gc_task_end.trigger()
    
    def _task_ended_callback(self):
        self._current_task = None
        self.node_callback()