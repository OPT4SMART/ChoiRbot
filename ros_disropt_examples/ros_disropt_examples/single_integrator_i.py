import rclpy
from rclpy.node import Node
from ros_disropt.integrator import SingleIntegrator
import numpy as np


def main():
    rclpy.init(args=None)

    steptime = 0.01

    integrator = SingleIntegrator(steptime)
    rclpy.spin(integrator)
