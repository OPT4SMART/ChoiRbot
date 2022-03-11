FROM ros:dashing

USER root

RUN . /opt/ros/dashing/setup.sh && \
    apt update -qq && apt install -y -q \
    build-essential \
    cmake \
    git \
    apt-transport-https \
    python3-colcon-common-extensions \
    software-properties-common \
    openssh-client \
    gnupg2 \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/* \

RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN pip3 install -U pip
RUN pip3 install -U setuptools

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    sudo apt-get install -y gazebo11 &&\
    rm -rf /var/lib/apt/lists/* \


# Clone all the necessary sources and install deps through rosdep.
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN groupadd -g 1000 rbccps && \
    useradd -d /home/rbccps -s /bin/bash -m rbccps -u 1000 -g 1000 && \
    usermod -aG sudo rbccps && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER rbccps
RUN mkdir -p /home/rbccps/mrs_ws/src
ENV HOME /home/rbccps
WORKDIR /home/rbccps/mrs_ws/
COPY src /home/rbccps/mrs_ws/src/


RUN colcon build --symlink-install

RUN cd /home/rbccps/mrs_ws/src/ && \
    pip3 install -r requirements.txt && \
    pip3 install -r requirements_disropt.txt && \
    pip3 install --no-deps disropt && \



RUN echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc


