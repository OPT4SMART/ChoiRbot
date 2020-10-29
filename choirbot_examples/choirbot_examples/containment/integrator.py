import rclpy
from choirbot.integrator import SingleIntegrator


def main():
    rclpy.init()

    frequency = 100

    integrator = SingleIntegrator(frequency)

    rclpy.spin(integrator)
    rclpy.shutdown()
