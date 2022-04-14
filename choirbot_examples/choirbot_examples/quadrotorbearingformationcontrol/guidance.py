import rclpy
from choirbot.guidance.distributed_control import BearingFormationControlGuidance
import time


def main():
    rclpy.init()

    frequency = 100
    prop_gain = 1.0
    deriv_gain = 5.0

    guidance = BearingFormationControlGuidance(frequency, prop_gain, deriv_gain, 'pubsub', 'odom')
    
    guidance.get_logger().info('Waiting for 5 seconds to let all nodes be ready')
    time.sleep(5)

    rclpy.spin(guidance)
    rclpy.shutdown()
