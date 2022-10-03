import rclpy
from choirbot.guidance.rl import SingleAgentRLPolicyGuidance, SingleUnicycleTensorflowRLController
import time


def main():
    rclpy.init()
    
    frequency = 100.0
    policy_model = None # TODO

    controller = SingleUnicycleTensorflowRLController(policy_model)
    guidance = SingleAgentRLPolicyGuidance(frequency, controller, 'pubsub', 'odom')

    guidance.get_logger().info('Waiting for 5 seconds to let all nodes be ready')
    time.sleep(5)

    rclpy.spin(guidance)
    rclpy.shutdown()
