import rclpy
from choirbot.guidance.distributed_control import ContainmentGuidance


def main():
    rclpy.init()
    
    frequency = 100.0
    gain = 0.1

    guidance = ContainmentGuidance(frequency, gain, 'pubsub', 'odom')

    rclpy.spin(guidance)
    rclpy.shutdown()
