from ros_disropt.communicators import ROSCommunicator
from disropt.agents import Agent
from disropt.algorithms import Consensus
from itertools import chain
import numpy as np
import time

def ros_consensus(id, N, neigh):
    
    # compute weights (metropolis hastings)
    w = 1/(len(neigh) + 1)
    weights = {i:w for i in chain(neigh,[id])}

    ################

    communicator = ROSCommunicator(id, N, neigh)
    agent = Agent(in_neighbors=neigh, in_weights=weights, communicator=communicator)

    time.sleep(1)
    print('Agent {} starting'.format(id))

    np.random.seed(id)

    n = 1
    x0 = np.random.rand(n, 1)*10
    algorithm = Consensus(agent=agent,
                        initial_condition=x0)
    
    print('Agent {}: initial condition {}'.format(id, x0.flatten()))
    
    algorithm.run(iterations=100, verbose = True)

    print('Agent {}: solution {}'.format(id, algorithm.get_result().flatten()))
    time.sleep(5)