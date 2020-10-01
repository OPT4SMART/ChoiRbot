from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from ros_disropt_interfaces.action import PositionAction
from threading import Event
from geometry_msgs.msg import Point
import numpy as np

from ..utils.position_getter import pose_subscribe
from .. import Pose


class Planner(Node):

    def __init__(self, pos_handler: str=None, pos_topic: str=None):
        super().__init__('planner', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.current_pose = Pose(None, None)
        self.subscription = pose_subscribe(pos_handler, pos_topic, self,
            self.current_pose, self.pose_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Planner {} started'.format(self.agent_id))
        self._goalreached_event = Event()
        self.checkdistance_gc = self.create_guard_condition(self.check_distance)
        self.goal_point = None

    def pose_callback(self):
        self.checkdistance_gc.trigger()

    def check_distance(self):
        if self.goal_point is not None:
            if np.linalg.norm(self.current_pose.position[:-1]-self.goal_point) < 0.05:
                self.get_logger().info("Goal reached - current position: {}".format(self.current_pose.position[:-1]))
                self._goalreached_event.set()


class PointToPointPlanner(Planner):

    def __init__(self, pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self._action_server = ActionServer(
            self,
            PositionAction,
            'positionaction',
            self.execute_callback)
        self.tocontrol_publisher = self.create_publisher(Point, 'goal', 10)

    def execute_callback(self, goal_handle):
        action_goal = goal_handle.request
        target_position = np.array(action_goal.goal_position)
        self.get_logger().info('Moving robot to position {}'.format(target_position))
        self.goal_point = np.copy(target_position)

        msg = Point()
        msg.x = target_position[0]
        msg.y = target_position[1]
        # msg.z = target_position[2]
        msg.z = 0.0
        self.tocontrol_publisher.publish(msg)

        goal_handle.succeed()
        result = PositionAction.Result()

        self._goalreached_event = Event()
        self._goalreached_event.wait()
        self.goal_point = None

        final_position = np.copy(self.current_pose.position)
        self.get_logger().info('Robot moved at position {}'.format(final_position))
        result.final_position = list(final_position)
        return result
