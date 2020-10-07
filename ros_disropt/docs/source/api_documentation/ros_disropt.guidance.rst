======================
Guidance Level
======================

The guidance level is a control layer providing high-level commands to the lower layers.
Here is a list of the implemented guidance scenarios.

.. toctree::
    Distributed Control <ros_disropt.guidance.distributed_control>
    Distributed Task Optimization <ros_disropt.guidance.task>
    Distributed Model Predictive Control <ros_disropt.guidance.mpc>

======================
Base Guidance classes
======================

Here we provide the documentation for the base Guidance classes that must be extended
for the various scenarios.

Guidance
----------------------

.. autoclass:: ros_disropt.guidance.Guidance
   :members:
   :undoc-members:
   :show-inheritance:

OptimizationGuidance
----------------------

.. autoclass:: ros_disropt.guidance.OptimizationGuidance
   :members:
   :undoc-members:
   :show-inheritance:

======================
Optimization Threads
======================

Optimization threads are an attribute of :class:`OptimizationGuidance`
and are used to solve optimization problems or to run distributed optimization
algorithms on a separate thread.

OptimizationThread
----------------------

.. autoclass:: ros_disropt.guidance.optimization_thread.OptimizationThread
   :members:
   :exclude-members: run
   :undoc-members:
   :show-inheritance: