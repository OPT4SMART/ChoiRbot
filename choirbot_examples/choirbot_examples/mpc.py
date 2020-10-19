import rclpy
import numpy as np
from math import sqrt
from choirbot.guidance.mpc import MPCGuidance

def main():
    rclpy.init(args=None)

    # initialize MPC guidance class
    sample_time = 1.0 # seconds

    pos_handler = 'pubsub'
    pos_topic = 'odom'

    guidance = MPCGuidance(sample_time, pos_handler, pos_topic)
    N = guidance.n_agents

    # system matrices
    A = np.eye((1, 1))
    B = np.eye((1, 1))*sample_time
    C = np.zeros((2*N*(N-1), 1))
    D = np.zeros((2*N*(N-1), 1))

    # fill C matrix
    counter = 0
    for i in range(N):
        for j in range(N):
            # skip if j <= i
            if j <= i:
                continue

            # fill either with normal sign (i) or with reversed sign (j)
            if i == guidance.agent_id:
                C[counter*2:(counter+1)*2] = np.array([[1], [-1]])

            if j == guidance.agent_id:
                C[counter*2:(counter+1)*2] = np.array([[-1], [1]])

            # increase counter
            counter += 1

    # cost matrices
    Q = np.eye((1, 1))
    R = np.eye((1, 1))

    # local constraints (state)
    E = np.array([[1], [-1]])
    f = 10 * np.ones((2, 1))

    # local constraints (input)
    G = np.array([[1], [-1]])
    h = np.ones((2, 1))

    # coupling constraints
    lhs_coupling = 0.6 * np.ones((2*N*(N-1), 1))

    # mechanism for trajectory continuation
    def traj_continuation(x_traj):
        # we consider an LQR terminal controller u = Kx
        P = (1 + sqrt(1 + 4 / sample_time**2))/2
        K = - sample_time * P / (1 + P * sample_time**2)

        # get last state
        x_end = x_traj[:, -1]

        # apply terminal controller
        x_new = (A + B @ K) @ x_end

        # compute output
        y_new = C@x_new

        return y_new.flatten()

    # initialize MPC scenario
    prediction_horizon = 10
    system_matrices = {'A': A, 'B': B, 'C': C, 'D': D}
    cost_matrices = {'state': Q, 'input': R}
    local_constraints = {'x_matrix': E, 'x_vector': f, 'u_matrix': G, 'u_vector': h}
    coupling_constraints = {'vector': lhs_coupling}

    guidance.initialize(prediction_horizon, system_matrices, cost_matrices,
        traj_continuation, local_constraints, coupling_constraints)

    # start
    rclpy.spin(guidance)

    rclpy.shutdown()
