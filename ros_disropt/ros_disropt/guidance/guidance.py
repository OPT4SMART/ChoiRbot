from rclpy.node import Node
from typing import List
from .. import Pose

from ..utils.position_getter import pose_subscribe
from . import RobotData
from ..communicators import ROSCommunicator


class Guidance(Node):

    def __init__(self, agent_id: int, N: int, in_neigh: List[int], out_neigh: List[int], # data: RobotData,
                 pose_handler: str=None, pose_topic: str=None):
        super().__init__('agent_{}_guidance'.format(agent_id))
        self.agent_id = agent_id
        self.agent_count = N
        self.in_neighbors = in_neigh
        self.out_neighbors = out_neigh
        # self.data = data
        self.current_pose = Pose(None, None)

        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose)
        self.communicator = self.instantiate_communicator()

    def instantiate_communicator(self):
        return ROSCommunicator(self.agent_id, self.agent_count, self.in_neighbors)
