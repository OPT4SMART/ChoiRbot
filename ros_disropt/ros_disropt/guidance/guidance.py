from rclpy.node import Node
from .. import Pose

from ..utils.position_getter import pose_subscribe
from . import RobotData
from ..communicators import ROSCommunicator


class Guidance(Node):

    def __init__(self, # data: RobotData,
                 pose_handler: str=None, pose_topic: str=None):
        super().__init__('guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.agent_count = self.get_parameter('N').value
        self.in_neighbors = self.get_parameter('in_neigh').value
        self.out_neighbors = self.get_parameter('out_neigh').value
        # self.data = data
        self.current_pose = Pose(None, None)

        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose)
        self.communicator = self.instantiate_communicator()

    def instantiate_communicator(self):
        return ROSCommunicator(self.agent_id, self.agent_count, self.in_neighbors)
