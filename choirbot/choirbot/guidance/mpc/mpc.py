import numpy as np
from typing import Callable
from geometry_msgs.msg import Vector3

from ...communicator import TimeVaryingCommunicator
from ...optimizer import MPCOptimizer
from ..guidance import OptimizationGuidance
from ..optimization_thread import OptimizationThread


class MPCGuidance(OptimizationGuidance):
    # Simplifications/assumptions:
    # - objective is regulation (not tracking)
    # - cost function is quadratic without affine terms
    # - system state is current position
    # - system input is Vector3 published in 'velocity' topic
    # - no disturbances
    # - no output y
    # - sets X, U, Z described by inequalities
    # - at the beginning, globally feasible output trajectories can obtained with u=0 identically
    # - no terminal constraint
    # - no terminal cost

    def __init__(self, sampling_period: float, pose_handler: str=None, pose_topic: str=None):
        super().__init__(MPCOptimizer(), MPCOptimizationThread, pose_handler, pose_topic)
        self.sampling_period = sampling_period
        self.timer = self.create_timer(sampling_period, self.control)
        self.ctrl_publisher = self.create_publisher(Vector3, 'velocity', 1)
        self.system_matrices = None
        self.traj_continuation = None
        self.output_trajectories = {}
        self.prediction_horizon = None
        self.can_control = False
        self.iteration = 0
        self.get_logger().info('Guidance {} started'.format(self.agent_id))
    
    def _instantiate_communicator(self):
        # communicator must have differentiated topics
        return TimeVaryingCommunicator(self.agent_id, self.n_agents, self.in_neighbors,
            out_neighbors=self.out_neighbors, differentiated_topics=True)
    
    def initialize(self, prediction_horizon: int, system_matrices: dict,
        cost_matrices: dict, traj_continuation: Callable,
        coupling_constraints: dict, local_constraints: dict=None):

        # initialize local variables
        self.system_matrices = system_matrices
        self.traj_continuation = traj_continuation
        self.prediction_horizon = prediction_horizon

        # initialize optimization scenario
        self.optimizer.initialize_scenario(self.agent_id, prediction_horizon,
            system_matrices, cost_matrices, coupling_constraints, local_constraints)

        # mark class as ready
        self.can_control = True
    
    def control(self):
        # skip if not ready
        if not self.can_control or self.current_pose.position is None:
            return

        # mark class as busy
        self.can_control = False

        # initialize output trajectory (only the first time)
        if not self.output_trajectories:
            self.get_logger().info('First-time initialization of output trajectory')
            self.initialize_output_trajectory()

        # gather trajectories at agent 0
        self.collect_trajectories()

        # receive trajectories from agent i-1
        if self.agent_id != 0:
            traj = self.communicator.neighbors_receive([self.agent_id-1])
            self.output_trajectories = traj[self.agent_id-1]
        
        # create and solve local optimal control problem
        self.optimizer.create_opt_control_problem(self.current_pose.position[0], self.output_trajectories)
        self.get_logger().info('Solving optimal control problem')
        self.optimization_thread.optimize()
    
    def _optimization_ended(self):
        # get resulting trajectories (from 0 to T-1)
        state_traj, input_traj, output_traj = self.optimizer.get_result()

        # update set of output trajectories
        self.output_trajectories[self.agent_id] = output_traj

        # send trajectories to agent i+1
        if self.agent_id != self.n_agents-1:
            self.communicator.neighbors_send(self.output_trajectories, [self.agent_id+1])

        # apply control input and shift horizon
        self.get_logger().info('Applying control and shifting horizon')
        self.send_input(input_traj[:, 0])
        self.shift_horizon(state_traj)
        self.communicator.current_label += 1 # increase label

        # mark class as ready
        self.can_control = True
        self.iteration += 1
    
    def initialize_output_trajectory(self):
        # shortcut variables
        A = self.system_matrices['A']
        C = self.system_matrices['C']
        T = self.prediction_horizon

        # initialize output trajectory with zeros
        y_traj = np.zeros((C.shape[0], T))

        # fill first entry
        x = self.current_pose.position[0, None]
        y_traj[:, 0] = C @ x

        # fill from 1 to T-1
        for t in range(1, T):
            x = A @ x
            y_traj[:, t] = C @ x
        
        # save trajectory
        self.output_trajectories[self.agent_id] = y_traj
    
    def collect_trajectories(self):
        if self.agent_id == 0:
            # receive output trajectories from other agents
            traj = self.communicator.neighbors_receive(list(range(1, self.n_agents)))
            
            # store received trajectories
            for i in range(1, self.n_agents):
                self.output_trajectories[i] = traj[i]
        else:
            # send output trajectory to agent 0
            self.communicator.neighbors_send(self.output_trajectories[self.agent_id], [0])
    
    def shift_horizon(self, state_traj):
        # delete first element of output trajectory
        self.output_trajectories[self.agent_id] = \
            np.delete(self.output_trajectories[self.agent_id], (0), axis=1)
        
        # extend local output trajectory
        output_cont = self.traj_continuation(state_traj)
        self.output_trajectories[self.agent_id] = \
            np.append(self.output_trajectories[self.agent_id], output_cont, axis=1)
    
    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = 0.0
        msg.z = 0.0

        self.ctrl_publisher.publish(msg)


class MPCOptimizationThread(OptimizationThread):

    def do_optimize(self):
        self.optimizer.optimize()