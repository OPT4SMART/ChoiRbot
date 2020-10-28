.. _installation:

Installation
===================

| The **ChoiRbot** toolbox currently supports ROS 2 Dashing Diademata.
| Please refer to the `ROS 2 website <https://index.ros.org/doc/ros2/>`_ for a comprehensive tutorial on how to install ROS 2.

If you do not have a ROS 2 workspace run on a terminal:

.. code-block:: bash

	mkdir -p ~/dev_ws/src
	cd ~/dev_ws/src

Then, you can clone the package repository:

.. code-block:: bash

	git clone https://github.com/OPT4SMART/ChoiRbot.git .
	
Finally, simply build your workspace:

.. code-block:: bash

	cd ~/dev_ws
	colcon build --symlink-install

**ChoiRbot** requires a set of Python packages that can be installed by run, inside the ``src`` directory of your workspace:

.. code-block:: bash

	pip install -r requirements.txt

If you are interested in running distributed optimization procedures, you also need the `DISROPT package <https://github.com/OPT4SMART/disropt>`_. You can install it by running:

.. code-block:: bash

	pip install -r requirements_disropt.txt
	pip install --no-deps disropt

You can also install disropt by directly running ``pip install disropt``. However, this would results in the installation of additional packages that are not required by **ChoiRbot**.