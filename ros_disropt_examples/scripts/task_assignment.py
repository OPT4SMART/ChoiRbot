from ros_disropt.scenario import TaskOptimizer
from disropt.agents import Agent
from disropt.algorithms import Consensus
from itertools import chain
import numpy as np
import time


def task_assignment(id, N, neigh, tasklist, taskindices, starting):

    task_opt = TaskOptimizer(id, N, tasklist, taskindices, starting, in_neighbors=neigh)
    ################
    task_opt.instantiate_algorithm()
    time.sleep(1)

    print("Agent {} A,b are {}".format(id, task_opt.problemConstraints()))
    print('Agent {} starting'.format(id))

    task_opt.optimize(iterations=100)
    print('Agente {} has basis {}'.format(id, task_opt.algorithm.B))
    print('Agent {} will do task {}'.format(id, task_opt.retrieve_results()))
