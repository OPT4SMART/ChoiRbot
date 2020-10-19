from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from typing import Callable
from choirbot_interfaces.action import PositionAction
from choirbot_interfaces.msg import PositionTask
from choirbot_interfaces.srv import PositionTaskService

class TaskExecutor:

    service = None
    node = None
    _current_task = None
    
    def initialize(self, node: Node):
        self.node = node
    
    def execute_async(self, task, callback: Callable):
        raise NotImplementedError

class PositionTaskExecutor(TaskExecutor):

    service = PositionTaskService
    _node_callback = None
    _position_client = None

    def initialize(self, node: Node):
        super().initialize(node)

        # create position action client
        self._position_client = ActionClient(
            node,
            PositionAction,
            'positionaction')

        # wait for action server
        self.node.get_logger().info('Waiting for action server')
        self._position_client.wait_for_server()
    
    def execute_async(self, task: PositionTask, callback: Callable):
        # store reference to callback
        self._node_callback = callback

        # prepare goal object
        goal_msg = PositionAction.Goal()
        goal_msg.goal_position = task.coordinates

        # send goal and prepare callback
        send_goal_future = self._position_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._goal_ended_callback)
    
    def _goal_ended_callback(self, future):
        # extract status
        result = future.result()
        status = result.status

        # execute node callback only on success
        if status == GoalStatus.STATUS_SUCCEEDED and self._node_callback:
            self._node_callback()