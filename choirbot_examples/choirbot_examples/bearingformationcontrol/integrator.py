import rclpy
from choirbot.integrator import DoubleIntegrator


def main():
    rclpy.init()
    print("integrator")
    frequency = 100.0
    integrator = DoubleIntegrator(frequency)

    rclpy.spin(integrator)
    rclpy.shutdown()
