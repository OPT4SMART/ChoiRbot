import rclpy
from choirbot.guidance.distributed_control import FormationControlGuidance


def main():
    rclpy.init()

    frequency = 100
    gain = 0.1

    guidance = FormationControlGuidance(frequency, gain, 'pubsub', 'odom')

    rclpy.spin(guidance)
    rclpy.shutdown()
