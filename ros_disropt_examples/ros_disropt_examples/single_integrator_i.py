import rclpy
from rclpy.node import Node
from ros_disropt.integrator import SingleIntegrator
import numpy as np


def main():
    rclpy.init(args=None)

    freq = 100

    integrator = SingleIntegrator(freq)
    rclpy.spin(integrator)
