import numpy as np
from typing import Dict
from .optimizer import Optimizer
from disropt.functions import Variable, QuadraticForm, AffineForm
from disropt.problems import QuadraticProblem


class MPCOptimizer(Optimizer):

    robot_id: int
    prediction_horizon: int
    system_matrices: dict
    coupling_constraints: dict
    _opt_problem: QuadraticProblem
    _opt_sol: np.ndarray
    _obj_func: QuadraticForm
    _basic_constraints: list
    _sizes: tuple
    _x: AffineForm
    _u: AffineForm
    
    def initialize_scenario(self, robot_id: int, prediction_horizon: int, system_matrices: dict,
        cost_matrices: dict, coupling_constraints: dict, local_constraints: dict=None):
        # save information
        self.prediction_horizon = prediction_horizon
        self.system_matrices = system_matrices
        self.coupling_constraints = coupling_constraints
        self.robot_id = robot_id

        # shorhands
        A = system_matrices['A']
        B = system_matrices['B']
        T = prediction_horizon
        E = local_constraints['x_matrix']
        f = local_constraints['x_vector']
        G = local_constraints['u_matrix']
        h = local_constraints['u_vector']
        Q = cost_matrices['state']
        R = cost_matrices['input']
        n, m = self._sizes = (A.shape[0], B.shape[1])

        # create optimization variables
        z = Variable((T+1)*n + T*m) # complete optimization variable
        self._x = np.vstack((np.eye((T+1)*n), np.zeros((T*m, (T+1)*n)))) @ z # state portion of optimization variable
        self._u = np.vstack((np.zeros(((T+1)*n, T*m)), np.eye(T*m))) @ z # input portion of optimization variable

        # initialize objective function and constraints
        self._obj_func = 0
        self._basic_constraints = []

        for t in range(T):
            # extract optimization variables corresponding to x(t), u(t), x(t+1)
            x_t  = np.eye(n, (T+1)*n, t*n).T @ self._x
            u_t  = np.eye(m, T*m, t*m).T @ self._u
            x_tp = np.eye(n, (T+1)*n, (t+1)*n).T @ self._x

            # build local constraints
            self._basic_constraints.append(x_tp == A.T @ x_t + B.T @ u_t) # dynamics
            self._basic_constraints.append(E.T @ x_t <= f) # state
            self._basic_constraints.append(G.T @ u_t <= h) # input

            # add terms to objective function
            self._obj_func += QuadraticForm(x_t, Q) + QuadraticForm(u_t, R)
    
    def create_opt_control_problem(self, initial_condition: np.ndarray, output_trajectories: Dict[int, np.ndarray]):
        # shorthands
        C = self.system_matrices['C']
        D = self.system_matrices['D']
        T = self.prediction_horizon
        b = self.coupling_constraints['vector']
        n, m = self._sizes

        # create a copy of output_trajectories without the current robot
        trajectories = output_trajectories.copy()
        if self.robot_id in trajectories:
            del trajectories[self.robot_id]

        # extract optimization variable corresponding to x(0)
        x_0 = np.eye(n, (T+1)*n, 0).T @ self._x

        # populate constraint list with basic constraints and set initial condition
        constraints = self._basic_constraints.copy()
        constraints.append(x_0 == initial_condition)

        for t in range(T):
            # extract optimization variables corresponding to x(t) and u(t)
            x_t  = np.eye(n, (T+1)*n, t*n).T @ self._x
            u_t  = np.eye(m, T*m, t*m).T @ self._u

            # build output constraints
            sum_traj_t = sum(traj[:, t] for traj in trajectories.values())[:, None]
            constraints.append(C.T @ x_t + D.T @ u_t + sum_traj_t <= b)
        
        # initialize problem
        self._opt_problem = QuadraticProblem(self._obj_func, constraints)

    def optimize(self):
        self._opt_sol = self._opt_problem.solve()

    def get_result(self):
        # shorthands
        T = self.prediction_horizon
        C = self.system_matrices['C']
        D = self.system_matrices['D']
        n, m = self._sizes

        # extract state and input trajectories
        x_traj = self._opt_sol[0 : (T+1)*n].reshape((n, T+1), order='F')
        u_traj = self._opt_sol[-T*m:].reshape((m, T), order='F')

        # compute output trajectory
        y_traj = np.zeros((C.shape[0], T))

        for t in range(T):
            y_traj[:, t] = C @ x_traj[:, t] + D @ u_traj[:, t]

        return x_traj, u_traj, y_traj
    
    def get_cost(self):
        return self._obj_func.eval(self._opt_sol)
