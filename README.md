# ChoiRbot [![Documentation Status](https://readthedocs.org/projects/choirbot/badge/?version=latest)](https://choirbot.readthedocs.io/en/latest/?badge=latest)
[**Website**](https://opt4smart.github.io/ChoiRbot/)
| [**Reference docs**](https://choirbot.readthedocs.io/en/latest/)
| [**Installation**](#requirements-and-installation)
| [**Getting started**](#getting-started)

| :warning: Information for end users |
|:------------------------------------|
| Documentation pages are currently being uploaded. The latest version will be available soon. |

**ChoiRbot** is a ROS 2 toolbox developed within the excellence research program ERC in the project [OPT4SMART](http://opt4smart.dei.unibo.it).
**ChoiRbot** provides a comprehensive set of libraries to execute complex distributed multi-robot tasks, such as model predictive control and task assignment, either in simulation or experimentally. The toolbox focuses on networks of heterogeneous robots without a central coordinator. It provides utilities for the solution of distributed optimization problems and can be also used to implement distributed feedback laws. Specifically, the package allows you to

- Encode distributed optimization and control algorithms
- Perform peer-to-peer communications among robots
- Develop planning and control schemes
- Connect with external motion capture hardware (see also our [ROS 2 Vicon Bridge](https://github.com/OPT4SMART/ros2-vicon-receiver))
- Run experiments on your robotic fleet
- Perform realistic simulations with [Gazebo](http://gazebosim.org) and visualize data with [RVIZ](https://github.com/ros2/rviz)

## Requirements and Installation
**ChoiRbot** requires ROS 2 Dashing Diademata to be installed on your system.

It relies on

* numpy
* scipy
* recordclass
* dill
* disropt (optional, but required for several features)

Please, refer to the [installation page](https://opt4smart.github.io/ChoiRbot/installation) for a more detailed installation guide.

To install the toolbox, first source your ROS 2 installation. Then create a ROS 2 workspace and, inside the `src` directory, run:
```
git clone https://github.com/OPT4SMART/ChoiRbot.git .
```

Then, from the parent directory execute:
```
colcon build --symlink-install
```

## Getting started
* [Quick start](https://choirbot.readthedocs.io/en/latest/quick_start/index.html)
* [Examples](https://choirbot.readthedocs.io/en/latest/examples/index.html)

## Citing **ChoiRbot**
If you are you using **ChoiRbot** in research work to be published, please cite the accompanying paper

```
@article{testa2021choirbot,
    title={ChoiRbot: A ROS 2 toolbox for cooperative robotics},
    author={Testa, Andrea and Camisa, Andrea and Notarstefano, Giuseppe},
    journal={IEEE Robotics and Automation Letters},
    volume={6},
    number={2},
    pages={2714--2720},
    year={2021},
    publisher={IEEE}
}
```

## Contributors
**ChoiRbot** is developed by
[Andrea Testa](https://andrea-testa.github.io),
[Andrea Camisa](https://www.unibo.it/sitoweb/a.camisa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)

## Acknowledgements
This result is part of a project that has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement No 638992 - OPT4SMART).

<p style="text-align:center">
  <img src="docs/source/_static/logo_ERC.png" width="200" />
  <img src="docs/source/_static/logo_OPT4Smart.png" width="200" /> 
</p>
