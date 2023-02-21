from .visualizer import Visualizer
from visualization_msgs.msg import Marker
import numpy as np
from typing import Callable

class SimpleVisualizer(Visualizer):

    def __init__(self, visualization_topic: str= '/visualization_marker', update_frequency: int= 25,
            pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None,):
        super().__init__(update_frequency, pose_handler, pose_topic, pose_callback)
        
        self.publisher_ = self.create_publisher(Marker, visualization_topic, 1)
        self.initialize_marker_data()
        
    def initialize_marker_data(self):
        self.scale = 0.2*np.ones(3)
        self.type = Marker.SPHERE
        self.color = np.array([1.0, 0.0, 0.0, 1.0])

    def publish_data(self):
        if self.current_pose.position is not None:
            msg = Marker()
            msg.pose.position.x = self.current_pose.position[0]
            msg.pose.position.y = self.current_pose.position[1]
            msg.pose.position.z = self.current_pose.position[2]

            msg.pose.orientation.x = self.current_pose.orientation[0]
            msg.pose.orientation.y = self.current_pose.orientation[1]
            msg.pose.orientation.z = self.current_pose.orientation[2]
            msg.pose.orientation.w = self.current_pose.orientation[3]

            msg.ns = 'robots'
            msg.id = self.agent_id

            msg.type = self.type

            msg.header.frame_id = 'my_frame'

            msg.scale.x = self.scale[0]
            msg.scale.y = self.scale[1]
            msg.scale.z = self.scale[2]

            msg.color.r = self.color[0]
            msg.color.g = self.color[1]
            msg.color.b = self.color[2]
            msg.color.a = self.color[3]

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.action = Marker.ADD

            self.publisher_.publish(msg)