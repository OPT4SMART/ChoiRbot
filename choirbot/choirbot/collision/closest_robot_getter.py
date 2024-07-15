
from rclpy.node import Node

import numpy as np

from .. import Pose
from ..utils.position_getter import pose_subscribe

import dill
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

from typing import Callable


class ClosestRobotGetter(Node):
    def __init__(self, sensing_distance: float = 3.0):

        super().__init__('closest_robots_getter',
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.sensing_distance = sensing_distance
        self.first_id = self.get_parameter('first_id').value
        self.last_id = self.get_parameter('last_id').value
        self.node_frequency = self.get_parameter('freq').value

        # initialize pose subscription
        self.subscriptions_pose_list = {}
        self.agents_pose = {i:Pose(None, None,None, None) for i in range(self.first_id, self.last_id)}

        # initialize closest robots list
        self.closest_robots_list = {i:{} for i in range(self.first_id, self.last_id)}
        
        for i in range(self.first_id, self.last_id):
            handler = 'pubsub'
            topic = '/agent_{}/odom'.format(i)
            callback = None
            self.subscriptions_pose_list[i] = pose_subscribe(handler, topic, self, self.agents_pose[i], callback)

        self.timer = self.create_timer(1.0/self.node_frequency, self.check_closest_robots)

        # initialize publishers
        self.closest_robots_publishers = {}
        for i in range(self.first_id, self.last_id):
            self.closest_robots_publishers[i] = self.create_publisher(ByteMultiArray, f'agent_{i}/closest_robots', 10)

        self.get_logger().info(f'ClosestRobotGetter_{self.first_id}-{self.last_id} started')


    def check_closest_robots(self):

        for i in range(self.first_id, self.last_id):
            for j in range(i+1, self.last_id):
                if i == j:
                    continue

                if self.agents_pose[i].position is None or self.agents_pose[j].position is None:
                    return
                
                dist = np.linalg.norm(self.agents_pose[i].position - self.agents_pose[j].position)
                if dist < self.sensing_distance:
                    if j not in self.closest_robots_list[i]:
                        # add new closest robot
                        self.closest_robots_list[i].update({j: self.agents_pose[j]})

                        # order the list by distance
                        self.closest_robots_list[i] = dict(sorted(self.closest_robots_list[i].items(), key=lambda item: np.linalg.norm(self.agents_pose[i].position - item[1].position)))

                        # send the closest robots to i-th agent
                        self.send_closest_robots(i)

                        # same for j-th agent
                        self.closest_robots_list[j].update({i: self.agents_pose[i]})
                        self.closest_robots_list[j] = dict(sorted(self.closest_robots_list[j].items(), key=lambda item: np.linalg.norm(self.agents_pose[j].position - item[1].position)))
                        self.send_closest_robots(j)
                    else:
                        # just update the content
                        self.closest_robots_list[i][j] = self.agents_pose[j]
                        self.send_closest_robots(i)

                        self.closest_robots_list[j][i] = self.agents_pose[i]
                        self.send_closest_robots(j)
                else:
                    if j in self.closest_robots_list[i]:
                        self.closest_robots_list[i].pop(j)
                        self.send_closest_robots(i)

                        self.closest_robots_list[j].pop(i)
                        self.send_closest_robots(j)


    def send_closest_robots(self, agent_id):
        """
        Send closest robots poses to i-th agent

        Args:
            agent_id (int): agent id
        """
        # convert obj to string
        data = dill.dumps(self.closest_robots_list[agent_id])


        # prepare message
        msg = ByteMultiArray()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = f'closest_robots_{agent_id}'
        msg.layout.dim[0].size = len(data)
        msg.layout.dim[0].stride = len(data)
        msg.data = [bytes([x]) for x in data]

        self.closest_robots_publishers[agent_id].publish(msg)


class ClosestRobotGetterVicon(ClosestRobotGetter):
    def __init__(self, sensing_distance: float = 3.0, vicon_ids: list = [], robot_name: str='tb', pose_callback: Callable=None):
        super().__init__(sensing_distance)

        self.vicon_ids = vicon_ids
        self.robot_name = robot_name

        if len(self.vicon_ids) != (self.last_id - self.first_id):
            raise RuntimeError('Number of agents does not match the vicon_ids list length.')

        for i in range(self.first_id, self.last_id):
            self.destroy_subscriber(self.subscriptions_pose_list[i])

        vicon_index = 0
        for i in range(self.first_id, self.last_id):
            handler = 'vicon'
            topic = f'/{self.robot_name}{self.vicon_ids[vicon_index]}/odom'
            callback = pose_callback
            self.subscriptions_pose_list[i] = pose_subscribe(handler, topic, self, self.agents_pose[i], callback)
