import rclpy
from choirbot.collision import UnicycleCollisionAvoidance
from choirbot.collision.safety_filters import SingleIntegratorCBF

from geometry_msgs.msg import Twist

def main():
    rclpy.init()

    frequency = 50.0            # Hz
    distance = 0.3              # m
    gamma = 10.0                # safety parameter
    max_velocity = 0.5          # [m/s] NOTE: These value is the maximum velocity for the single integrator dynamic.
    max_braking_velocity = 0.5  # [m/s] NOTE: These value is the maximum braking velocity for the single integrator dynamic.
    
    safety_filter = SingleIntegratorCBF(
        distance = distance, 
        gamma = gamma, 
        max_velocity = max_velocity,
        max_braking_velocity = max_braking_velocity
        )

    collision_avoidance = UnicycleCollisionAvoidance(
        pose_handler='pubsub', 
        pose_topic='odom', 
        pose_callback=None, 
        node_frequency=frequency, 
        topic_msg=Twist,
        topic_name='cmd_vel', 
        safety_filter=safety_filter
        )

    rclpy.spin(collision_avoidance)
    rclpy.shutdown()
