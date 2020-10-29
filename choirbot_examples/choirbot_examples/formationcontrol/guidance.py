import rclpy
from choirbot.guidance.distributed_control import FormationControlGuidance
import time


def main():
    rclpy.init()

    frequency = 100
    gain = 0.1

    guidance = FormationControlGuidance(frequency, gain, 'pubsub', 'odom')
    
    guidance.get_logger().info('Waiting for 5 seconds to let all nodes be ready')
    time.sleep(5)

    rclpy.spin(guidance)
    rclpy.shutdown()
