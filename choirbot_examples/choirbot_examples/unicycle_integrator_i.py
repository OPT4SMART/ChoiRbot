import rclpy
from rclpy.node import Node
from choirbot.integrator import UnicycleIntegrator
import numpy as np


def main():
    rclpy.init(args=None)

    freq = 100

    integrator = UnicycleIntegrator(freq)
    rclpy.spin(integrator)
