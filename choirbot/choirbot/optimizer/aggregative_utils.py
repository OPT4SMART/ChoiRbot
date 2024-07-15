import numpy as np
from typing import Union, Callable

from disropt.agents import Agent
from disropt.algorithms import Algorithm
from disropt.functions import AbstractFunction
from disropt.problems import Problem

from copy import deepcopy

class AggregativeProblem(Problem):
    """A local part of an aggregative problem. 

    Args:
        objective_function (AbstractFunction, optional): Local objective function. Defaults to None.
        constraints (list, optional): Local constraints. Defaults to None.
        aggregative_function (AbstractFunction, optional): Local function contributing to aggregative variable. Defaults to None.

    Attributes:
        objective_function (Function): Objective function to be minimized
        constraints (AbstractSet or Constraint): Local constraints
        aggregative_function (Function): Local function contributing to aggregative variable
    """

    def __init__(self, objective_function: AbstractFunction = None,
                 aggregative_function: AbstractFunction = None,
                 **kwargs):
        super().__init__(objective_function=objective_function, **kwargs)

        if aggregative_function is not None:
            self.set_aggregative_function(aggregative_function)

    def set_aggregative_function(self, fn: AbstractFunction):
        """Set the aggregative function

        Args:
            fn: aggregative function

        Raises:
            TypeError: input must be a AbstractFunction 
        """
        if not isinstance(fn, AbstractFunction):
            raise TypeError("aggregative function must be a AbstractFunction")
        self.aggregative_function = fn




class AggregativeGradientTracking(Algorithm):
    """Aggregative Gradient Tracking Algorithm [...]_

    From the perspective of agent :math:`i` the algorithm works as follows. For :math:`k=0,1,\\dots`

    .. math::

        x_i^{k+1} & = x_i^k - \\alpha (\\nabla_1 f_i(x_i^k,s_i^k) + \\nabla\\phi_i(x_i^k)y_i^k) \\\\
        s_i^{k+1} & = \\sum_{j=1}^N w_{ij} s_j^k - [ \\phi_i (x_i^{k+1}) - \\phi_i (x_i^k)] 
        y_i^{k+1} & = \\sum_{j=1}^N w_{ij} y_j^k - [ \\nabla_2 f_i (x_i^{k+1},s_i^{k+1}) - \\nabla_2 f_i (x_i^k,s_i^k)]


    where :math:`x_i\\in\\mathbb{R}^n` and :math:`s_i\\in\\mathbb{R}^{n \\times n_{\\sigma}}` and :math:`y_i\\in\\mathbb{R}^{n_\\sigma}`. The weight matrix :math:`W=[w_{ij}]` must be doubly-stochastic. Extensions to other class of weight matrices :math:`W` are not currently supported.

    Args:
        agent (Agent): agent to execute the algorithm
        initial_condition (numpy.ndarray): initial condition for :math:`x_i`
        enable_log (bool): True for enabling log

    Attributes:
        agent (Agent): agent to execute the algorithm
        x0 (numpy.ndarray): initial condition
        x (numpy.ndarray): current value of the local solution
        s (numpy.ndarray): current value of the local aggregative tracker
        y (numpy.ndarray): current value of the local gradient tracker
        shape (tuple): shape of the variable
        s_neigh (dict): dictionary containing the local aggregative tracker of the (in-)neighbors
        y_neigh (dict): dictionary containing the local gradient tracker of the (in-)neighbors
        enable_log (bool): True for enabling log
    """

    def __init__(self, agent: Agent, initial_condition: np.ndarray, enable_log: bool = False):
        super(AggregativeGradientTracking, self).__init__(agent, enable_log)

        d = initial_condition.shape[0]
        # dimension sigma supposed equal to agent dimenrion
        d_sigma = d
        mask_x = np.hstack((np.eye(d),np.zeros((d_sigma,d_sigma)))).T
        mask_sigma = np.hstack((np.zeros((d,d)),np.eye(d_sigma))).T

        self.x0 = deepcopy(initial_condition)
        self.x = deepcopy(initial_condition)

        # Initialize trackers
        self.s0 = self.agent.problem.aggregative_function.eval(initial_condition)
        self.s = self.agent.problem.aggregative_function.eval(initial_condition)
        self.y = mask_sigma.T@self.agent.problem.objective_function.subgradient(np.vstack((initial_condition,self.s)))

        #Initialize gradients
        self.nabla_2 = mask_sigma.T@self.agent.problem.objective_function.subgradient(np.vstack((initial_condition,self.s)))
        self.nabla_phi = self.agent.problem.aggregative_function.jacobian(initial_condition).T

        self.s_neigh = {}
        self.y_neigh = {}

        # Post processing variables
        self.cost_list = []
        self.subgradient_list = []
        self.trackers_list = []
        self.aggregative_function_list = []
        self.position_list = []

    def _update_local_solution(self, x: np.ndarray, **kwargs):
        """update the local solution

        Args:
            x: new value

        Raises:
            TypeError: Input must be a numpy.ndarray
            ValueError: Incompatible shapes
        """
        if not isinstance(x, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")
        if x.shape != self. x0.shape:
            raise ValueError("Incompatible shapes")
        self.x = x

    def _update_local_aggregative_tracker(self, s: np.ndarray, **kwargs):
        """update the local aggregative tracker

        Args:
            s: new value

        Raises:
            TypeError: Input must be a numpy.ndarray
            ValueError: Incompatible shapes
        """
        if not isinstance(s, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")
        if s.shape != self. s0.shape:
            raise ValueError("Incompatible shapes")
        self.s = s

    def _update_local_gradient_tracker(self, y: np.ndarray, **kwargs):
        """update the local gradient tracker

        Args:
            y: new value

        Raises:
            TypeError: Input must be a numpy.ndarray
            ValueError: Incompatible shapes 
        """
        if not isinstance(y, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")
        if y.shape != self. s0.shape:
            raise ValueError("Incompatible shapes")
        self.y = y

    def iterate_run(self, stepsize: float, **kwargs):
        """Run a single iterate of the aggregative gradient tracking algorithm
        """

        d = self.x.shape[0]
        mask_x = np.hstack((np.eye(d),np.zeros((2,2)))).T
        mask_sigma = np.hstack((np.zeros((2,2)),np.eye(d))).T

        data_received = self.agent.neighbors_exchange([self.s,self.y])
        for neigh in data_received:
            self.s_neigh[neigh]       = deepcopy(data_received[neigh][0])
            self.y_neigh[neigh]       = deepcopy(data_received[neigh][1])
    
        s_kp = self.agent.in_weights[self.agent.id] * self.s
        y_kp = self.agent.in_weights[self.agent.id] * self.y

        for j in self.agent.in_neighbors:
            s_kp += self.agent.in_weights[j] * self.s_neigh[j]
            y_kp += self.agent.in_weights[j] * self.y_neigh[j]
        
        self.nabla_1 = mask_x.T@self.agent.problem.objective_function.subgradient(np.vstack((self.x,self.s)))
        self.nabla_phi = self.agent.problem.aggregative_function.jacobian(self.x).T

        x_kp = self.x - stepsize*(mask_x.T@self.agent.problem.objective_function.subgradient(np.vstack((self.x,self.s))) + self.agent.problem.aggregative_function.jacobian(self.x).T@self.y) 
        s_kp += self.agent.problem.aggregative_function.eval(x_kp) - self.agent.problem.aggregative_function.eval(self.x)

        old_nabla_2 = self.nabla_2
        self.nabla_2 = mask_sigma.T@self.agent.problem.objective_function.subgradient(np.vstack((x_kp,s_kp)))

        y_kp += self.nabla_2 - old_nabla_2

        self._update_local_solution(x_kp, **kwargs)
        self._update_local_aggregative_tracker(s_kp, **kwargs)
        self._update_local_gradient_tracker(y_kp, **kwargs)
    
    def run(self, iterations: int = 1000, stepsize: Union[float, Callable] = 0.1, verbose: bool=False, callback_iter: Callable=None) -> np.ndarray:
        """Run the aggregative gradient tracking algorithm for a given number of iterations

        Args:
            iterations: Number of iterations. Defaults to 1000.
            stepsize: If a float is given as input, the stepsize is constant. 
                                                        Default is 0.01.
            verbose: If True print some information during the evolution of the algorithm. Defaults to False.
        
        Raises:
            TypeError: The number of iterations must be an int
            TypeError: The stepsize must be a float
        
        Returns:
            return the sequence of estimates if enable_log=True.
        """
        if not isinstance(iterations, int):
            raise TypeError("The number of iterations must be an int")
        if not (isinstance(stepsize, float) or callable(stepsize)):
            raise TypeError("The stepsize must be a float or a function")

        if self.enable_log:
            dims = [iterations]
            for dim in self.x.shape:
                dims.append(dim)
            self.sequence = np.zeros(dims)
        
        for k in range(iterations):
            if not isinstance(stepsize, float):
                step = stepsize(k)
            else:
                step = stepsize

            self.iterate_run(stepsize=step)

            self.cost_list.append(self.get_cost())
            self.subgradient_list.append(self.get_subgradient())
            self.aggregative_function_list.append(self.get_aggregative_function())
            self.trackers_list.append(self.get_trackers())
            self.position_list.append(self.x)

            if callback_iter is not None:
                callback_iter()

            if self.enable_log:
                self.sequence[k] = self.x
            
            if verbose:
                if self.agent.id == 0:
                    print('Iteration {}'.format(k), end="\r")
        
        if self.enable_log:
            return self.sequence
        

    def get_result(self):
        """Return the actual value of x

        Returns:
            numpy.ndarray: value of x
        """
        # print('xdes',self.x[0,0],self.x[1,0])
        return self.x


    def get_cost(self):
        """
        Return the actual value of the cost function

        Returns:
            float: value of the cost function
        """
        opt_var = np.vstack((self.x, self.s))
        cost = self.agent.problem.objective_function.eval(opt_var)
        return float(cost)

    def get_cost_list(self):
        return self.cost_list
    
    def get_subgradient(self):
        """
        Return the actual value of the subgradient
        
        Returns:
            numpy.ndarray: value of the subgradient
        """
        arg_grad = np.vstack((self.x, self.s))
        sub_grad = self.agent.problem.objective_function.subgradient(arg_grad)
        return sub_grad

    def get_subgradient_list(self):
        return self.subgradient_list

    def get_trackers(self):
        return np.vstack((self.s, self.y))

    def get_trackers_list(self):
        return self.trackers_list
    
    def get_aggregative_function(self):
        return self.agent.problem.aggregative_function.eval(self.x)

    def get_aggregative_function_list(self):
        return self.aggregative_function_list

    def get_position_list(self):
        return self.position_list
