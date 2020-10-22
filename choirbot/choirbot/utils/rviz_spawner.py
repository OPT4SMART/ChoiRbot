from rclpy.node import Node
from .. import Pose
from .position_getter import pose_subscribe
from visualization_msgs.msg import Marker
#from rclpy.duration import Duration
import numpy as np



class RvizSpawner(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None):
        super().__init__('rviz', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 1)

        self.current_pose = Pose(None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose)
        self.timer = self.create_timer(0.1, self.spawn)
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()


    def spawn(self):
        
        current_pos = np.copy(self.current_pose.position)
        self.current_time = self.get_clock().now()
        if current_pos.ndim is not 0:
            msg = Marker()
            msg.pose.position.x = current_pos[0]
            msg.pose.position.y = current_pos[1]
            msg.pose.position.z = 0.0

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            msg.ns = 'robots'
            msg.id = self.agent_id

            msg.type = Marker.SPHERE

            msg.header.frame_id = 'my_frame'

            msg.scale.x = 0.2
            msg.scale.y = 0.2
            msg.scale.z = 0.2

            msg.color.r = 1.0
            msg.color.g = 0.0
            msg.color.b = 0.0
            msg.color.a = 1.0

            msg.header.stamp = self.current_time.to_msg()
            msg.action = Marker.ADD
            #duration = Duration()
            #duration.sec = 0
            #duration.nanosec = 1e6
            #msg.lifetime = duration

            self.publisher_.publish(msg)
        self.last_time = self.current_time