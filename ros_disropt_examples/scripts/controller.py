#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import rclpy
from rclpy.node import Node

from .turtlebot3_position_control import Turtlebot3PositionControl
from .turtlebot3_position_control_feedback import Turtlebot3Feedback


def main(args=None):
    rclpy.init(args=args)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(agent_id, agent_id)
    
    # turtlebot3_position_control = Turtlebot3PositionControl(agent_id, pos_handler, pos_topic)
    turtlebot3_position_control = Turtlebot3Feedback(agent_id, pos_handler, pos_topic)
    rclpy.spin(turtlebot3_position_control)

    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
