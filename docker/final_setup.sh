#!/bin/bash

# Clean up unnecessary webots_ros2 packages
rm -rf /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_core \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_epuck \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_mavic \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_tesla \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_tests \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_tiago \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_turtlebot \
       /home/$USERNAME/$WS_NAME/src/webots_ros2/webots_ros2_universal_robot
