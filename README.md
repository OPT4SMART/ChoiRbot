# ChoiRbot [![Documentation Status](https://readthedocs.org/projects/choirbot/badge/?version=latest)](https://choirbot.readthedocs.io/en/latest/?badge=latest)
[**Reference docs**](https://choirbot.readthedocs.io/en/latest/)
| [**Installation**](#requirements-and-installation)
| [**Getting started**](#getting-started)

| :warning: Information for end users |
|:------------------------------------|
| Documentation pages are currently being uploaded. The latest version will be available soon. |

**ChoiRbot** is a ROS 2 toolbox developed within the excellence research program ERC in the project [OPT4SMART](http://opt4smart.dei.unibo.it).
**ChoiRbot** provides a comprehensive set of libraries to execute complex distributed multi-robot tasks, such as model predictive control and task assignment, either in simulation or experimentally. The toolbox focuses on networks of heterogeneous robots without a central coordinator. It provides utilities for the solution of distributed optimization problems and can be also used to implement distributed feedback laws.

## Requirements and Installation
**ChoiRbot** requires ROS 2 Dashing Diademata to be installed on your system.

It relies on

* numpy
* scipy
* recordclass
* dill
* disropt (optional, but required for several features)

Please, refer to the [readthedocs installation page](https://choirbot.readthedocs.io/en/latest/installation/index.html) for a more detailed installation guide.

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
@article{testa2020choirbot,
      title={ChoiRbot: A ROS 2 Toolbox for Cooperative Robotics}, 
      author={Testa, Andrea and Camisa, Andrea and Notarstefano, Giuseppe},
      journal={arXiv},
      archivePrefix={arXiv},
      eprint={2010.13431},
      year={2020}
}
```

## Contributors
**ChoiRbot** is developed by
[Andrea Testa](https://www.unibo.it/sitoweb/a.testa),
[Andrea Camisa](https://www.unibo.it/sitoweb/a.camisa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)

## Acknowledgements
This result is part of a project that has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement No 638992 - OPT4SMART).

<p style="text-align:center">
  <img src="docs/source/_static/logo_ERC.png" width="200" />
  <img src="docs/source/_static/logo_OPT4Smart.png" width="200" /> 
</p>

## Instructions to bringup the docker environment(ROS 2 dashing with gazebo-11)

- Tested on docker version 20.10.7
- Requires docker compose v1.28.0 or higher


**Install the following packages if you have a dedicated nvidia graphics card**

-  Add the GPG key:
	
	```
	distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   	&& curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-	key add - \
   	&& curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   	```
  
- Install the nvidia-docker2 package 
  
  	```
  	sudo apt-get update
  	```
  	
  	```
  	sudo apt-get install -y nvidia-docker2
  	```
   
- Restart the Docker daemon
     
   	```
   	sudo systemctl restart docker
   	``` 
**Instructions to build the image and run the container**

- Place the Dockerfile, the docker-compose.yml file  and the docker-compose_gpu.yml file inside ~/your_workspace/. **NOTE:** Your entire src/ folder will be mounted as a volume in the container.

- To build the image:

	```
	./bringup_dashing_devel.sh -c build
	```
	
	NOTE:  This step might be time consuming.

- To run the container:
	```
	cd  ~/your_workspace/
	```

	```
	 ./bringup_dasing_devel.sh -c run
	 ```
	 
	 Wait for the bash script to execute. In another terminal 
	 
	 ```
	 docker-compose exec warehouse_sim /bin/bash
	 ```
