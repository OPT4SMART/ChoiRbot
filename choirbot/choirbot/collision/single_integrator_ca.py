from typing import Callable
from geometry_msgs.msg import Twist

from .collision_avoidance import CollisionAvoidance

from .safety_filters import SafetyFilterStrategy


class SingleIntegratorCollisionAvoidance(CollisionAvoidance):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, 
                 node_frequency: float = 10.0, topic_msg = Twist, topic_name: str = 'velocity', safety_filter: SafetyFilterStrategy = None):
        super().__init__(pose_handler, pose_topic, pose_callback, node_frequency, topic_msg, topic_name, safety_filter)
        self.desired_input_z = 0.0
    
    def get_state(self):
        state = self.current_pose.position[:2].tolist()
        return state
    
    def get_obstacles(self):
        return [pose.position[:2] for pose in self.closest_robots_poses]

    def get_desired_input(self, msg):
        self.desired_input = [msg.linear.x, msg.linear.y]
        self.desired_input_z = msg.linear.z

        
    def send_safe_input(self, safe_input):

        safe_input_msg = Twist()

        safe_input_msg.linear.x = safe_input[0]
        safe_input_msg.linear.y = safe_input[1]
        safe_input_msg.linear.z = self.desired_input_z
        self.publisher_safe_input.publish(safe_input_msg)
