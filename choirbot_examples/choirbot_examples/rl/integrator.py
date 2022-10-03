import rclpy
from choirbot.integrator import UnicycleIntegrator


def main():
    rclpy.init()

    frequency = 100

    integrator = UnicycleIntegrator(frequency)

    rclpy.spin(integrator)
    rclpy.shutdown()
