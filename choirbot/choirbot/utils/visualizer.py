from rclpy.node import Node
from .. import Pose
from .position_getter import pose_subscribe
from visualization_msgs.msg import Marker


class Visualizer(Node):

    def __init__(self, visualization_topic: str= '/visualization_marker', update_frequency: int= 25,
            pose_handler: str=None, pose_topic: str=None):
        super().__init__('visualizer', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        self.agent_id = self.get_parameter('agent_id').value
        self.publisher_ = self.create_publisher(Marker, visualization_topic, 1)
        
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose)
        self.timer = self.create_timer(1.0/update_frequency, self.publish_message)

    def publish_message(self):
        if self.current_pose.position is not None:
            msg = Marker()
            msg.pose.position.x = self.current_pose.position[0]
            msg.pose.position.y = self.current_pose.position[1]
            msg.pose.position.z = self.current_pose.position[2]

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

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.action = Marker.ADD

            self.publisher_.publish(msg)