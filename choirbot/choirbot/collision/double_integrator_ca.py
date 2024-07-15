from typing import Callable
from geometry_msgs.msg import Vector3

from .collision_avoidance import CollisionAvoidance

from .safety_filters import SafetyFilterStrategy

class DoubleIntegratorCollisionAvoidance(CollisionAvoidance):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, 
                 node_frequency: float = 10.0, topic_msg = Vector3, topic_name: str = 'acceleration', safety_filter: SafetyFilterStrategy = None):
        super().__init__(pose_handler, pose_topic, pose_callback, node_frequency, topic_msg, topic_name, safety_filter)
    
        self.desired_input_z = 0.0
        
    def get_state(self):
        state = self.current_pose.position[:2].tolist() + self.current_pose.velocity[:2].tolist()
        return state
    
    def get_obstacles(self):
        obstacles = [pose.position[:2].tolist() + pose.velocity[:2].tolist() for pose in self.closest_robots_poses]
        return obstacles
    
    def get_desired_input(self, msg):
        self.desired_input = [msg.x, msg.y]
        self.desired_input_z = msg.z

        
    def send_safe_input(self, safe_input):

        safe_input_msg = Vector3()
        
        safe_input_msg.x = safe_input[0]
        safe_input_msg.y = safe_input[1]
        safe_input_msg.z = self.desired_input_z
        self.publisher_safe_input.publish(safe_input_msg)