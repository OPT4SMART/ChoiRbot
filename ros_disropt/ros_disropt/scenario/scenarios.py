from ..communicators import ROSCommunicator
from threading import Event


class Scenario():
    def __init__(self, robot_id: int, network_size: id, in_neighbors: list=None, out_neighbors: list=None,  **kwargs):
        if not isinstance(in_neighbors, list):
            raise ValueError("in_neighbors must be an instance of list.")
        if out_neighbors is not None:
            if not isinstance(out_neighbors, list):
                raise ValueError("out_neighbors must be an instance of list.")
        if not isinstance(robot_id, int) or robot_id < 0:
            raise TypeError("robot_id must be a non-negative indexes")
        if not isinstance(network_size, int) or network_size < 0:
            raise TypeError("network_size must be a non-negative indexes")

        # initialize variables
        self.robot_id = robot_id
        self.network_size = network_size
        self.in_neighbors = in_neighbors
        self.out_neighbors = out_neighbors
        self.communicator = self.instantiate_communicator()

    def instantiate_communicator(self):
        return ROSCommunicator(self.robot_id, self.network_size, self.in_neighbors)

    def problemConstraints(self):
        pass

    def costFunction(self):
        pass

    def instantiate_algorithm(self):
        pass

    def optimize(self):
        pass

    def retrieve_results(self):
        pass


class Optimizer(Scenario):
    def __init__(self, robot_id: int, network_size: int, in_neighbors: list=None, out_neighbors: list=None,
                 resolution_strategy: str=None, event: Event=None, **kwargs):
        super(Optimizer, self).__init__(robot_id, network_size, in_neighbors, out_neighbors)
        if resolution_strategy is not None:
            if not isinstance(resolution_strategy, str):
                raise TypeError("resolution_strategy must be a string")
        self.resolution_strategy = resolution_strategy
        self.run_optimizaton = True
        self.algorithm = None
        self.event = event

    def discover_strategy(self):
        pass

    def run_algorithm(self):
        self.algorithm.run()
