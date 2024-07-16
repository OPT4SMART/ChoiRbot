import numpy as np
from typing import Callable
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty

from ...communicator import TimeVaryingCommunicator
from ...optimizer import AggregativeOptimizer
from ..guidance import OptimizationGuidance
from ..optimization_thread import OptimizationThread


class AggregativeGuidance(OptimizationGuidance):

    def __init__(self, optimizer: AggregativeOptimizer, 
                 freq_reference: float=10.0, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None,
                 msg_topic: str='position', msg_type=Vector3, 
                 ):
        super().__init__(optimizer=optimizer, 
                         thread_t=AggregativeOptimizationThread, 
                         pose_handler=pose_handler, 
                         pose_topic=pose_topic,
                         pose_callback=pose_callback,
                         )
                
        self.freq_reference = freq_reference
        self.msg_topic = msg_topic
        self.msg_type = msg_type
        self.timer = self.create_timer(1/self.freq_reference, self.send_reference)
        self.reference_publisher = self.create_publisher(self.msg_type, self.msg_topic, 1)

        self.optimization_ended = False

        # triggering mechanism to start optimization
        self.opt_trigger_subscription = self.create_subscription(
                Empty, '/experiment_trigger', self.start_optimization, 10)
        
        self.get_logger().info('Aggregative Guidance {} started'.format(self.agent_id))
    
    def _instantiate_communicator(self):
        # communicator must have differentiated topics
        return TimeVaryingCommunicator(self.agent_id, self.n_agents, self.in_neighbors,
            out_neighbors=self.out_neighbors, differentiated_topics=True)
        
    def _optimization_ended(self):
        self.optimization_ended = True
        self.get_logger().info('Optimization ended')

        self.save_results()

    def save_results(self):
        # save results
        raise NotImplementedError

    def initialize(self):
        # initialize optimization scenario
        self.optimizer.initialize(self)
    
    def start_optimization(self, _):
        self.get_logger().info('Optimization triggered')

        # launch optimization
        self.optimization_thread.optimize()

    def get_initial_condition(self):
        """
        TODO
        return np.array() of shape (agent_dim,1)
        """
        raise NotImplementedError
    

    def get_intruder(self):
        """
        TODO
        return np.array() of shape (agent_dim,1)
        """
        raise NotImplementedError

    def get_target(self):
        """
        TODO
        return np.array() of shape (agent_dim,1)
        """
        raise NotImplementedError
    
    def send_reference(self):
        # skip if not ready
        if self.current_pose.position is None or self.optimizer.algorithm is None or self.optimization_ended:
            return
        
        # get reference
        x_des = self.optimizer.get_result()

        msg = self.generate_reference_msg(self.msg_type, x_des)

        self.reference_publisher.publish(msg)

    def generate_reference_msg(self, msg_type, x_des):
        """
        TODO
        input: 
            msg_type: msg_type
            x_des: np.array() of shape (agent_dim,1)
        return msg: msg_type
        """
        raise NotImplementedError
    
class AggregativeOptimizationThread(OptimizationThread):

    def do_optimize(self):
        self.optimizer.create_problem()
        self.optimizer.optimize()