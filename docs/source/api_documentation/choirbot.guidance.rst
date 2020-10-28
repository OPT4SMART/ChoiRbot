======================
Team Guidance Layer
======================

The guidance level is a control layer providing high-level commands to the lower layers.

Base Guidance classes
======================

Here we provide the documentation for the base Guidance classes that must be extended
for the various scenarios.

Guidance
----------------------

.. autoclass:: choirbot.guidance.Guidance
   :members:
   :undoc-members:

OptimizationGuidance
----------------------

.. autoclass:: choirbot.guidance.OptimizationGuidance
   :members:
   :undoc-members:
   :show-inheritance:

Optimization Threads
======================

Optimization threads are an attribute of :class:`OptimizationGuidance`
and are used to solve optimization problems or to run distributed optimization
algorithms on a separate thread.

OptimizationThread
----------------------

.. autoclass:: choirbot.guidance.optimization_thread.OptimizationThread
   :members:
   :exclude-members: run
   :undoc-members:
   :show-inheritance:

Complex Guidance classes
=========================

Here is a list of the implemented guidance scenarios.

.. toctree::
   :maxdepth: 1

   Distributed Control <choirbot.guidance.distributed_control>
   Distributed Task Optimization <choirbot.guidance.task>
   Distributed Model Predictive Control <choirbot.guidance.mpc>