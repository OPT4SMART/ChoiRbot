#!/bin/bash

DEV_NAME=opt4smart
WS_NAME=choirbot_ws

# Clean up unnecessary webots_ros2 packages
rm -rf /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_core \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_epuck \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_mavic \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tesla \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tests \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tiago \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_turtlebot \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_universal_robot