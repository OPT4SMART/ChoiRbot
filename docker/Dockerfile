ARG UBUNTU_VERSION=20.04
ARG WEBOTS_VERSION=R2022a

FROM cyberbotics/webots:${WEBOTS_VERSION}-ubuntu${UBUNTU_VERSION}

ENV DEV_NAME=opt4smart
ENV ROS_DISTRO=foxy
ENV WS_NAME=choirbot_ws

##########################################################
# Solves bugs of cyberboics/webots:R2022a image
##########################################################
RUN rm /etc/apt/sources.list.d/cuda.list && \
    apt-key del 7fa2af80  && \
    apt-get update && apt-get install -y --no-install-recommends wget  && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb  && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    rm cuda-keyring_1.0-1_all.deb
##########################################################

##########################################################
# Install basic packages
##########################################################
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
        curl \
        gedit \
        git \
        iputils-ping \
        locales \
        nano \
        net-tools \
        nano \
        python3-pip \
        software-properties-common \
        sudo \
        vim \
        xterm \
    && rm -rf /var/lib/apt/lists/*
##########################################################

##########################################################
# Install ROS 2
##########################################################
RUN locale-gen en_US en_US.UTF-8  && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8  && \
    export LANG=en_US.UTF-8 && \    
    add-apt-repository universe  && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \    
    apt update -qq && \
    apt install -yq --no-install-recommends\
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-gazebo-* \
        ros-${ROS_DISTRO}-rqt-graph \
        ros-${ROS_DISTRO}-rviz2 \
        python3-colcon-common-extensions 
##########################################################

##########################################################
# Create group and user
##########################################################
RUN groupadd -g 1000 ${DEV_NAME} && \
     useradd -d /home/${DEV_NAME} -s /bin/bash -m ${DEV_NAME} -u 1000 -g 1000 && \
     usermod -aG sudo ${DEV_NAME} && \
     echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
##########################################################

##########################################################
# Create bridge folder
##########################################################
RUN mkdir -p /home/${DEV_NAME}/${WS_NAME}
##########################################################

##########################################################
# Install ChoiRbot and Disropt requirements
##########################################################
RUN pip3 install -U pip setuptools==58.2.0 tqdm
COPY ./choirbot_requirements.txt /home/${DEV_NAME}
COPY ./final_setup.sh /home/${DEV_NAME}
WORKDIR /home/${DEV_NAME}
RUN pip3 install --upgrade pip && \
    pip3 install -r choirbot_requirements.txt && \
    pip3 install --no-deps disropt 

##########################################################
# Clone Turtlebot3 simulation package
##########################################################
RUN mkdir -p /home/${DEV_NAME}/turtlebot3_ws/src/
WORKDIR /home/${DEV_NAME}/turtlebot3_ws/src/
RUN git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
##########################################################

##########################################################
# Install qpSWIFT
##########################################################
WORKDIR /home/${DEV_NAME}/
RUN git clone https://github.com/qpSWIFT/qpSWIFT && \
    cd /home/${DEV_NAME}/qpSWIFT/python && \
    sudo python3 setup.py install && \
    sudo rm -rf /home/${DEV_NAME}/qpSWIFT
##########################################################

##########################################################
# Copy and install final setup script
##########################################################
COPY ./final_setup.sh /home/${DEV_NAME}/
RUN chmod +x /home/${DEV_NAME}/final_setup.sh
##########################################################

##########################################################
# Set useful .bashrc aliases
##########################################################
USER ${DEV_NAME}
RUN echo 'PROMPT_DIRTRIM=1' >> /home/${DEV_NAME}/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> /home/${DEV_NAME}/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/home/$DEV_NAME/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models"' >> /home/${DEV_NAME}/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/${DEV_NAME}/.bashrc && \
    echo 'export ROS_DOMAIN_ID=100' >> /home/${DEV_NAME}/.bashrc && \
    echo 'alias ros2-init="source /opt/ros/${ROS_DISTRO}/setup.bash"' >> /home/${DEV_NAME}/.bashrc  && \
    echo 'alias ros2-build="colcon build --symlink-install"' >> /home/${DEV_NAME}/.bashrc  && \
    echo 'alias ros2-source="source install/setup.bash"' >> /home/${DEV_NAME}/.bashrc  && \
    echo 'alias ros2-clean="rm -rf build/ log/ install/"' >> /home/${DEV_NAME}/.bashrc  && \
    echo 'cat<<TF' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo 'Welcome to...' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '          ,+°/.\°+,   ___  ' >> /home/${DEV_NAME}/.bashrc && \
    echo '        \/\_/\_/\_/\,+ * \ ' >> /home/${DEV_NAME}/.bashrc && \
    echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' >> /home/${DEV_NAME}/.bashrc && \
    echo '       /_/ ------- \_\              ' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo '-------   CCCC   H   H    OOO    I   RRRR    BBBB     OOO   TTTTT   --- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '------  C       H   H   O   O   I   R   R   B   B   O   O    T    ----- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '-----  C       HHHHH   O   O   I   RRRR    BBBB    O   O    T   ------ ' >> /home/${DEV_NAME}/.bashrc && \
    echo '----   C      H   H   O   O   I   R  R    B   B   O   O    T   ------- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '---     CCC  H   H    OOO    I   R    R  BBBB     OOO     T  --------- ' >> /home/${DEV_NAME}/.bashrc && \
    echo '' >> /home/${DEV_NAME}/.bashrc && \
    echo 'TF' >> /home/${DEV_NAME}/.bashrc
##########################################################


WORKDIR /home/${DEV_NAME}/${WS_NAME}
ENTRYPOINT /home/${DEV_NAME}/final_setup.sh && /bin/bash
