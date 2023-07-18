FROM tiryoh/ros2-desktop-vnc:foxy

RUN apt update -qq && apt install -y -q apt-utils && apt upgrade -y -q && apt-get install -y -q \
    build-essential \
    cmake \
    git \
    apt-transport-https \
    software-properties-common \
    locales \
    openssh-client \
    python3-pip \
    wget \
    ros-foxy-gazebo-* \
    ros-foxy-dynamixel-sdk \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3

RUN pip3 install -U pip setuptools

USER root

RUN groupadd -g 1000 ubuntu && \
     useradd -d /home/ubuntu -s /bin/bash -m ubuntu -u 1000 -g 1000 && \
     usermod -aG sudo ubuntu && \
     echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ubuntu

RUN mkdir -p /home/ubuntu/dev_ws/src

COPY . /home/ubuntu/dev_ws/src

WORKDIR /home/ubuntu/dev_ws/src
RUN     pip3 install --upgrade pip && \
        pip3 install -r requirements.txt && \
    	pip3 install -r requirements_disropt.txt && \
    	pip3 install --no-deps disropt 
    
WORKDIR /home/ubuntu/dev_ws/
RUN source /opt/ros/foxy/setup.sh && \ 
    colcon build --symlink-install

RUN mkdir -p /home/ubuntu/turtlebot3_ws/src/
WORKDIR /home/ubuntu/turtlebot3_ws/src/
RUN git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
WORKDIR /home/ubuntu/turtlebot3_ws/
RUN source /opt/ros/foxy/setup.sh && \ 
    colcon build --symlink-install

RUN echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/home/ubuntu/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models"' >> ~/.bashrc
