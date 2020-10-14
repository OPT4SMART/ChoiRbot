.. _installation:

Installation
===================

| The **ChoiRbot** package currently supports ROS 2 Dashing Diademata.
| Please refer to the `ROS 2 website <https://index.ros.org/doc/ros2/>`_ for a comprehensive tutorial on how to install ROS 2.

If you do not have a ROS 2 workspace run on a terminal:

.. code-block:: bash

	mkdir -p ~/dev_ws/src
	cd ~/dev_ws/src

Then, you can clone the package repository:

.. code-block:: bash

	git clone ..........
	
Finally, simply build your workspace:

.. code-block:: bash

	cd ~/dev_ws
	colcon build --symlink-install
