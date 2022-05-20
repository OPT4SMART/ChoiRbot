import numpy as np
from rclpy.qos import QoSProfile
from rclpy.node import Node
from typing import Callable
from rclpy.callback_groups import CallbackGroup


def pose_subscribe(handler: str, topic: str, node: Node, dest_pose: 'Pose',
    callback: Callable=None, callback_group: CallbackGroup=None, qos: QoSProfile=QoSProfile(depth=10)):
    if not handler in ['pubsub', 'tf', 'tf2', 'vicon']:
        raise TypeError('odom_handler must be pubsub, tf, tf2 or vicon')

    if handler == 'pubsub':
        from nav_msgs.msg import Odometry
        subscription = node.create_subscription(Odometry, topic,
                                                lambda x: odom_callback(x, dest_pose, callback),
                                                qos, callback_group=callback_group)
    elif handler == 'vicon':
        from vicon_receiver.msg import Position
        subscription = node.create_subscription(Position, topic,
                                                lambda x: vicon_callback(x, dest_pose, callback),
                                                qos, callback_group=callback_group)
    
    return subscription

def odom_callback(msg, dest_pose: 'Pose', callback: Callable):
    point = msg.pose.pose.position
    quat  = msg.pose.pose.orientation
    linear   = msg.twist.twist.linear
    angular  = msg.twist.twist.angular

    dest_pose.position = np.array([point.x, point.y, point.z])
    dest_pose.orientation = np.array([quat.x, quat.y, quat.z, quat.w])
    dest_pose.velocity = np.array([linear.x, linear.y, linear.z])
    dest_pose.angular = np.array([angular.x, angular.y, angular.z])

    if callback is not None:
        callback()

def vicon_callback(msg, dest_pose: 'Pose', callback: Callable):
    dest_pose.position = np.array([msg.x_trans, msg.y_trans, msg.z_trans])/1000
    dest_pose.orientation = np.array([msg.x_rot, msg.y_rot, msg.z_rot, msg.w])

    if callback is not None:
        callback()