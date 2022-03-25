With **ChoiRbot**, you can easily develop your new distributed optimization or control scheme on a real network of cooperating robots. Take a look at the following implementations!

#### Multi-Robot Pickup-and-Delivery

In the video, a heterogeneous team composed of seven Turtlebot 3 ground robots and two Crazyflie 2 aerial robots receive a total of 10 pickup with 10 associated delivery requests (only a subset of them is shown to avoid cluttering the image). The robots execute a [distributed resource allocation algorithm](https://arxiv.org/abs/2104.02415) to self-assign the tasks by communicating with neighboring robots and then move toward the designated locations.

<p align="center"><iframe width="560" height="315" src="https://www.youtube.com/embed/NwqzIEBNIS4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe></p>

If you are interested, take a look at the [accompaining paper](https://arxiv.org/abs/2104.02415).

#### Dynamic Task Allocation

In the video, four mobile robots address a dynamic task assignment scenario. In the proposed scheme robots dynamically self-assign the tasks by communicating with neighboring robots, plan their trajectories and move toward the assigned tasks. As soon as a task is served a new one appears, so
that robots have to re-organize their allocations according to the new problem data.

<p align="center"><iframe width="560" height="315" src="https://www.youtube.com/embed/uii1BjFGqMM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe></p>

Try this example on your own! Follow the installation procedure [here](/installation) and run

    source install/setup.bash
    ros2 launch choirbot_examples taskassignment.launch.py

A detailed explanation can be found [at this link](https://choirbot.readthedocs.io/en/latest/examples/dynamic_task_assignment.html) and in our [accompaining paper](https://arxiv.org/pdf/2010.13431.pdf).

#### Distributed Formation Control

In the following video, six mobile robots, simulated through Gazebo, execute a formation control task. Their goal is to cooperatively draw an hexagon leveraging local computation and communication with neighboring robots.

<p align="center"><iframe width="560" height="315" src="https://www.youtube.com/embed/VIIXpzPTfPU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe></p>

Try this example on your own! Follow the installation procedure [here](/installation) and run

    source install/setup.bash
    ros2 launch choirbot_examples formationcontrol.launch.py

A detailed explanation can be found [at this link](https://choirbot.readthedocs.io/en/latest/examples/formation_control.html) and in our [accompaining paper](https://arxiv.org/pdf/2010.13431.pdf).