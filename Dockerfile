FROM tiryoh/ros2-desktop-vnc:dashing

USER root
RUN apt update -qq && apt-get install -y -q \
    build-essential \
    cmake \
    git \
    apt-transport-https \
    software-properties-common \
    locales \
    openssh-client \
    python3-pip \
    wget \
    ros-galactic-turtlebot3* \
    ros-galactic-gazebo-ros \
    ros-galactic-rviz2

RUN pip3 install -U pip setuptools

RUN groupadd -g 1000 choirbot && \
    useradd -d /home/choirbot -s /bin/bash -m choirbot -u 1000 -g 1000 && \
    usermod -aG sudo choirbot && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER choirbot
RUN mkdir -p /home/choirbot/dev_ws/src
ENV HOME /home/choirbot
WORKDIR /home/choirbot/dev_ws/
COPY . /home/choirbot/dev_ws/src

RUN cd /home/choirbot/dev_ws/src/ && \
    pip3 install -r requirements.txt && \
    pip3 install -r requirements_disropt.txt && \
    pip3 install --no-deps disropt && \
    . /opt/ros/dashing/setup.sh && \
    cd /home/choirbot/dev_ws/ && \
    colcon build --symlink-install

RUN echo 'export PATH=${PATH}:/home/choirbot/.local/bin' >> ~/.bashrc
RUN echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/opt/ros/dashing/share/turtlebot3_gazebo/models"' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
