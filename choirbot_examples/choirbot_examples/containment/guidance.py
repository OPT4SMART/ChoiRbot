import rclpy
from choirbot.guidance.distributed_control import ContainmentGuidance
import time


def main():
    rclpy.init()
    
    frequency = 100.0
    gain = 0.1

    guidance = ContainmentGuidance(frequency, gain, 'pubsub', 'odom')

    guidance.get_logger().info('Waiting for 5 seconds to let all nodes be ready')
    time.sleep(5)

    rclpy.spin(guidance)
    rclpy.shutdown()
