FROM ros:dashing

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
    ros-dashing-turtlebot3* \
    ros-dashing-gazebo-ros* \
    ros-dashing-rviz2 \
    && pip3 install -U pip setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN groupadd -g 1000 rbccps && \
    useradd -d /home/rbccps -s /bin/bash -m rbccps -u 1000 -g 1000 && \
    usermod -aG sudo rbccps && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER rbccps
RUN mkdir -p /home/rbccps/mrs_ws/src
ENV HOME /home/rbccps
WORKDIR /home/rbccps/mrs_ws/
COPY . /home/rbccps/mrs_ws/src

RUN cd /home/rbccps/mrs_ws/src/ && \
    pip3 install -r requirements.txt && \
    pip3 install -r requirements_disropt.txt && \
    pip3 install --no-deps disropt && \
    . /opt/ros/dashing/setup.sh && \
    cd /home/rbccps/mrs_ws/ && \
    colcon build --symlink-install

RUN echo 'export PATH=${PATH}:/home/rbccps/.local/bin' >> ~/.bashrc
RUN echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/opt/ros/dashing/share/turtlebot3_gazebo/models"' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
