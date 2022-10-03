from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy as np
import tensorflow as tf
from scipy.spatial.transform import Rotation as R

from .rl_guidance import RLController


def read_unicycle_state(pose):
    # read position on plane
    state = np.zeros(3)
    state[0:2] = pose.position[0:2]

    # read heading
    state[2] = R.from_quat(pose.orientation).as_euler('xyz')[2]
    state[2] %= 2 * np.pi # ensure it is between 0 and 2*pi
    return state

class SingleUnicycleTensorflowRLController(RLController):

    def __init__(self, policy_model):
        self.policy = policy_model

    def _initialize_publisher(self, input_topic):
        return self.node.create_publisher(Twist, input_topic, 1)
    
    def evaluate_input(self, pose):
        state = read_unicycle_state(pose)
        state_tensor = tf.convert_to_tensor(state[None, :])
        action_tensor = self.policy(state_tensor)
        action = action_tensor.numpy().squeeze()
        return action
    
    def send_input(self, u):
        msg = Twist()
        msg.linear.x  = u[0]
        msg.angular.z = u[1]
        self.publisher_.publish(msg)

class DistributedUnicycleTensorflowRLController(SingleUnicycleTensorflowRLController):
    
    def initialize(self, node: Node, input_topic: str):
        super().initialize(node, input_topic)

        # read list of neighbors and id of current agent
        self.neighbors = node.get_parameter('in_neigh').value
        agent_id = node.get_parameter('agent_id').value

        # add current agent if not already in the list
        if agent_id not in self.neighbors:
            self.neighbors.append(agent_id)

        # sort the list
        self.neighbors.sort()

    def evaluate_input(self, neigh_data):
        policy_input = []
        for i in self.neighbors:
            state_i = read_unicycle_state(neigh_data[i])
            state_i_tensor = tf.convert_to_tensor(state_i[None, :])
            policy_input.append(state_i_tensor)
        action_tensor = self.policy(tuple(policy_input))
        action = action_tensor.numpy().squeeze()
        return action
