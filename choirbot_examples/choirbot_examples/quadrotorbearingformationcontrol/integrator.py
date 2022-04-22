import rclpy
from choirbot.integrator import QuadrotorIntegrator


def main():
    rclpy.init()
    print("integrator")
    frequency = 100.0
    integrator = QuadrotorIntegrator(frequency)

    rclpy.spin(integrator)
    rclpy.shutdown()
