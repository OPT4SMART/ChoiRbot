from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from choirbot_interfaces.action import PositionAction
from threading import Event, Lock
from geometry_msgs.msg import Point
import numpy as np

from ..utils import pose_subscribe
from ..utils import OrEvent
from .. import Pose


class Planner(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None):
        super().__init__('planner', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self,
            self.current_pose, self.pose_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Planner {} started'.format(self.agent_id))        

    def pose_callback(self):
        pass


class PointToPointPlanner(Planner):

    def __init__(self, goal_tolerance: float=0.05, pose_handler: str=None, pose_topic: str=None):
        super().__init__(pose_handler, pose_topic)
        self._goalreached_event = Event()
        self._abort_event = Event()
        self._lock = Lock()
        self.goal_point = None
        self._tolerance = goal_tolerance
        self._action_server = ActionServer(
            self,
            PositionAction,
            'positionaction',
            self.execute_callback)
        self.tocontrol_publisher = self.create_publisher(Point, 'goal', 10)
    
    def pose_callback(self):
        # check distance to goal point
        if self.goal_point is not None and not self._goalreached_event.is_set():
            if np.linalg.norm(self.current_pose.position[:-1]-self.goal_point) < self._tolerance:
                self.get_logger().info("Goal reached - current position: {}".format(self.current_pose.position[:-1]))
                self._goalreached_event.set()

    def execute_callback(self, goal_handle):
        # abort any previous action
        if self.goal_point is not None:
            with self._lock:
                self._abort_event.set()
                self._abort_event = Event()
        
        # read goal point
        action_goal = goal_handle.request
        self.goal_point = np.array(action_goal.goal_position)
        self.get_logger().info('Moving robot to position {}'.format(self.goal_point))

        # send message to controller
        self.send_to_controller()

        # wait for either event (abort or success)
        with self._lock:
            abort_event = self._abort_event
        OrEvent(self._goalreached_event, abort_event).wait()

        if self._goalreached_event.is_set(): # success

            # reset goal
            self.goal_point = None

            # clear event
            self._goalreached_event.clear()

            # notify client
            goal_handle.succeed()

        else: # abort

            # notify client
            goal_handle.abort()
        
        # return result
        result = PositionAction.Result()
        result.final_position = list(self.current_pose.position)

        return result

    def send_to_controller(self):
        msg = Point()
        msg.x = self.goal_point[0]
        msg.y = self.goal_point[1]
        msg.z = self.goal_point[2]
        self.tocontrol_publisher.publish(msg)


class TwoDimPointToPointPlanner(PointToPointPlanner):

    def send_to_controller(self):
        msg = Point()
        msg.x = self.goal_point[0]
        msg.y = self.goal_point[1]
        msg.z = 0.0
        self.tocontrol_publisher.publish(msg)
