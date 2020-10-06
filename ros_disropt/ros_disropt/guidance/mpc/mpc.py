import numpy as np
from typing import Callable
from geometry_msgs.msg import Vector3

from ...optimizer import MPCOptimizer
from ..guidance import OptimizationGuidance
from ..optimization_thread import OptimizationThread


class MPCGuidance(OptimizationGuidance):
    # Simplifications/assumptions:
    # objective is regulation (not tracking)
    # cost function is quadratic without affine terms
    # system state is current position
    # system input is Vector3 published in 'velocity' topic
    # no disturbances
    # no output y
    # sets X, U, Z described by inequalities
    # no terminal constraints
    # no terminal cost

    def __init__(self, sampling_period: float, pos_handler: str=None, pos_topic: str=None):
        super().__init__(MPCOptimizer(), MPCOptimizationThread, pos_handler, pos_topic)
        self.sampling_period = sampling_period
        self.timer = self.create_timer(sampling_period, self.control)
        self.ctrl_publisher = self.create_publisher(Vector3, 'velocity', 1)
        self.traj_continuation = None
        self.output_trajectories = None
        self.input_trajectory = None
        self.state_trajectory = None
        self.prediction_horizon = None
        self.can_control = False
        self.get_logger().info('Guidance {} started'.format(self.agent_id))
    
    def initialize(self, prediction_horizon: int, system_matrices: dict, cost_matrices: dict,
        output_trajectory: np.ndarray, traj_continuation: Callable,
        local_constraints: dict=None, coupling_constraints: dict=None):

        # initialize local variables
        self.traj_continuation = traj_continuation
        self.output_trajectories = {self.agent_id: output_trajectory}
        self.prediction_horizon = prediction_horizon

        # initialize optimization scenario
        self.optimizer.initialize_scenario(prediction_horizon, system_matrices, cost_matrices,
            local_constraints, coupling_constraints)
        
        # apply control input and shift horizon
        self.apply_control_and_shift_horizon()

        # mark class as ready
        self.can_control = True
    
    def control(self):
        # skip if not ready
        if not self.can_control or self.current_pose.position is None:
            return
        
        # mark class as busy
        self.can_control = False

        # gather trajectories at agent 0
        self.collect_trajectories()

        # if not agent 0, receive trajectories from agent i-1
        if self.agent_id != 0:
            traj = self.communicator.neighbors_receive([self.agent_id-1])
            self.output_trajectories = traj.values().pop()
        
        # create and solve local optimal control problem
        self.optimizer.create_opt_control_problem(self.current_pose.position, self.output_trajectories)
        self.optimization_thread.optimize()
    
    def optimization_ended(self):
        # get resulting output trajectory
        state_traj, input_traj, output_traj = self.optimizer.get_result()

        # store state/input trajectories
        self.state_trajectory = state_traj
        self.input_trajectory = input_traj

        # update set of output trajectories
        self.output_trajectories[self.agent_id] = output_traj

        # apply control input and shift horizon
        self.apply_control_and_shift_horizon()

        # mark class as ready
        self.can_control = True
    
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
    
    def apply_control_and_shift_horizon(self):
        # apply first control input
        self.send_input(self.input_trajectory[:, 0])

        # discard first input
        np.delete(self.input_trajectory, (0), axis=1)

        # extend local output trajectory
        output_cont = self.traj_continuation(self.state_trajectory)
        np.append(self.output_trajectories[self.agent_id], output_cont, axis=1)
    
    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.ctrl_publisher.publish(msg)


class MPCOptimizationThread(OptimizationThread):

    def do_optimize(self):
        self.optimizer.optimize()