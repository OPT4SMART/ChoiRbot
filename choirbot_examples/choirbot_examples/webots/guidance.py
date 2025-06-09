import rclpy
from choirbot.guidance.distributed_control import SimpleGuidance

def main():
    rclpy.init()

    freq_guidance = 2.0
    guidance = SimpleGuidance(freq_guidance, 'pubsub', 'odom')

    rclpy.spin(guidance)
    rclpy.shutdown()
