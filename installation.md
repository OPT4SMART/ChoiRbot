# Installation


In this page, we provide the installation procedure for the **ChoiRbot** package.


## Toolbox download and build

The **ChoiRbot** toolbox currently supports ROS 2 Dashing Diademata.
Please refer to the [ROS 2 website](https://index.ros.org/doc/ros2/) for a comprehensive
tutorial on how to install ROS 2. We suggest to perform the *Desktop Install* of ROS 2,
which provides useful tools such as RVIZ.

If you do not have a ROS 2 workspace run on a terminal:

	mkdir -p ~/dev_ws/src
	cd ~/dev_ws/src

To download **ChoiRbot**, clone the package repository:

	git clone https://github.com/OPT4SMART/ChoiRbot.git .
	
Finally, simply build your workspace:

	cd ~/dev_ws
	colcon build --symlink-install


## Installation of required Python packages

**ChoiRbot** requires a set of Python packages that can be installed by running
(inside the ``src`` directory of your workspace):

	pip3 install -r requirements.txt

If you are interested in running distributed optimization algorithms, you also need
the [DISROPT package](https://github.com/OPT4SMART/disropt).
You can install it by running:

	pip3 install -r requirements_disropt.txt
	pip3 install --no-deps disropt

You could also install disropt by directly running ``pip install disropt``. However,
this would automatically install additional packages (such as mpi4py) that are
not required by **ChoiRbot**.


## Installation of Gazebo and Turtlebot3 files

**ChoiRbot** allows yo to easily simulate a team of Turtlebot3 mobile robots.
You just need to perform the installation steps at this [link (PC Setup tab)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) and to clone the ROS 2 workspace provided [here (Gazebo Simulation tab)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).
Then, be sure that the .bashrc file contains the following lines (notice that the paths may need to be adjusted according to your current installation).

	export TURTLEBOT3_MODEL=burger
	source ~/turtlebot3_ws/install/setup.bash
	export ROS_DOMAIN_ID=30 #TURTLEBOT3
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models