import numpy as np
from rclpy.qos import QoSProfile

def position_subscribe(handler, topic, node, callback, callback_group=None, qos=QoSProfile(depth=10)):
    if not handler in ['pubsub', 'tf', 'tf2', 'vicon']:
        raise TypeError('odom_handler must be pubsub, tf, tf2 or vicon')

    if handler == 'pubsub':
        from nav_msgs.msg import Odometry
        subscription = node.create_subscription(Odometry, topic,
                                                lambda x: odom_callback(x, callback),
                                                qos, callback_group=callback_group)
    elif handler == 'vicon':
        from vicon_receiver.msg import Position
        subscription = node.create_subscription(Position, topic,
                                                lambda x: vicon_callback(x, callback),
                                                qos, callback_group=callback_group)
    
    return subscription

def odom_callback(msg, callback):
    point = msg.pose.pose.position
    quat  = msg.pose.pose.orientation

    position = np.array([point.x, point.y, point.z])
    orientation = np.array([quat.x, quat.y, quat.z, quat.w])

    callback(position, orientation)

def vicon_callback(msg, callback):
    position = np.array([msg.x_trans, msg.y_trans, msg.z_trans])/1000
    orientation = np.array([msg.x_rot, msg.y_rot, msg.z_rot, msg.w])

    callback(position, orientation)