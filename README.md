**ChoiRbot** is a ROS 2 toolbox developed within the excellence research program ERC in the project [OPT4SMART](http://opt4smart.dei.unibo.it).
**ChoiRbot** provides a comprehensive set of libraries to execute complex distributed multi-robot tasks, such as model predictive control and task assignment, either in simulation or experimentally. The toolbox mainly focuses on networks of heterogeneous robots without a central coordinator. It provides utilities for the solution of distributed optimization problems and can be also used to implement distributed feedback laws.

## Requirements and Installation
**ChoiRbot** requires ROS 2 Dashing Diademata to be installed on your system.

It relies on

* numpy
* scipy
* recordclass
* disropt (optional, but required for several features)

To install the toolbox, create a ROS 2 workspace and, inside the `src` directory, run:
```
git clone https://github.com/OPT4SMART/ChoiRbot.git .
```

Then, from the parent directory execute:
```
colcon build
```


## Constributors
**ChoiRbot** is developed by
[Andrea Testa](https://www.unibo.it/sitoweb/a.testa),
[Andrea Camisa](https://www.unibo.it/sitoweb/a.camisa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)

## Acknowledgements
This result is part of a project that has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement No 638992 - OPT4SMART).

<!-- <p style="text-align:center">
  <img src="docs/source/_static/logo_ERC.png" width="200" />
  <img src="docs/source/_static/logo_OPT4Smart.png" width="200" /> 
</p> -->