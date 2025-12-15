##########################################################
# Build of cyberbotics/webots:R2025a-ubuntu24.04
##########################################################
ARG BASE_IMAGE=nvidia/cuda:12.9.0-base-ubuntu24.04
FROM ${BASE_IMAGE} AS downloader

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_PACKAGE_PREFIX=

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes wget bzip2 && rm -rf /var/lib/apt/lists/ && \
    wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2 && \
    tar xjf webots-*.tar.bz2 && rm webots-*.tar.bz2

FROM ${BASE_IMAGE} AS webots_r2025a

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Install Webots runtime dependencies
RUN apt-get update && apt-get install --yes wget xvfb locales && rm -rf /var/lib/apt/lists/ && \
    wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh && \
    chmod +x linux_runtime_dependencies.sh && ./linux_runtime_dependencies.sh && rm ./linux_runtime_dependencies.sh && rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
COPY --from=downloader /webots /usr/local/webots/
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}

# Enable OpenGL capabilities
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# Set a user name to fix a warning
ENV USER root

# Set the locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Finally open a bash command to let the user interact
CMD ["/bin/bash"]

##########################################################
# Build choirbot:jazzy
##########################################################

ARG UBUNTU_VERSION=24.04
ARG WEBOTS_VERSION=R2025a

FROM webots_r2025a

ARG WS_NAME
ENV USERNAME=ubuntu
ENV ROS_DISTRO=jazzy

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
        htop \
    && rm -rf /var/lib/apt/lists/*
##########################################################

##########################################################
# Install ROS 2
##########################################################
RUN locale-gen en_US en_US.UTF-8  && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8  && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository universe -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    sudo dpkg -i /tmp/ros2-apt-source.deb
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-rqt-graph \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-webots-ros2-tests \
        ros-${ROS_DISTRO}-webots-ros2-turtlebot \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-laser-filters \
        python3-colcon-common-extensions \
        python3-rosdep
RUN rosdep init
##########################################################

##########################################################
# Create group and user
##########################################################
RUN usermod -aG sudo ${USERNAME} && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${USERNAME}
SHELL ["/bin/bash", "-c"]
RUN rosdep update
##########################################################

##########################################################
# Create bridge folder
##########################################################
RUN mkdir -p /home/${USERNAME}/${WS_NAME}
##########################################################

##########################################################
# Create python venv
##########################################################
RUN sudo apt update && sudo apt install -y python3-venv
RUN mkdir /home/${USERNAME}/venv
ENV VIRTUAL_ENV /home/${USERNAME}/venv
RUN python3 -m venv $VIRTUAL_ENV --system-site-packages

ENV PATH "$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH /home/${USERNAME}/venv/lib/python3.12/site-packages:$PYTHONPATH
RUN echo "source /home/${USERNAME}/venv/bin/activate" >> /home/${USERNAME}/.bashrc

##########################################################
# Install ChoiRbot and Disropt requirements
##########################################################
COPY ./docker/requirements.txt /home/${USERNAME}
WORKDIR /home/${USERNAME}
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r requirements.txt && \
    python3 -m pip install --no-deps disropt

##########################################################
# Clone Turtlebot3 simulation package
##########################################################
RUN mkdir -p /home/${USERNAME}/turtlebot3_ws/src
RUN git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git /home/${USERNAME}/turtlebot3_ws/src/ && \
    cd /home/${USERNAME}/turtlebot3_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src -y --ignore-src && \
    python3 -m colcon build --symlink-install && \
    echo "source /home/${USERNAME}/turtlebot3_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
##########################################################

##########################################################
# Install qpSWIFT
##########################################################
WORKDIR /home/${USERNAME}/
RUN git clone https://github.com/qpSWIFT/qpSWIFT && \
    cd /home/${USERNAME}/qpSWIFT/python && \
    sudo python3 setup.py install && \
    sudo rm -rf /home/${USERNAME}/qpSWIFT
##########################################################

##########################################################
# Copy and install final setup script
##########################################################
COPY ./docker/final_setup.sh /home/${USERNAME}/
RUN sudo chmod +x /home/${USERNAME}/final_setup.sh
##########################################################

##########################################################
# Set useful .bashrc aliases
##########################################################
RUN echo 'PROMPT_DIRTRIM=1' >> /home/${USERNAME}/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-init="source /opt/ros/$ROS_DISTRO/setup.bash"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-build="python3 -m colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-source="source install/setup.bash"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-clean="rm -rf build/ log/ install/"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-rosdep="rosdep install --from-paths src -y --ignore-src"' >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-pkg-create-py="ros2 pkg create --build-type ament_python --license Apache-2.0"' >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-pkg-create-cpp="ros2 pkg create --build-type ament_cmake --license Apache-2.0"' >> /home/${USERNAME}/.bashrc && \
    echo 'cat<<TF' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo 'Welcome to...' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '          ,+°/.\°+,   ___  ' >> /home/${USERNAME}/.bashrc && \
    echo '        \/\_/\_/\_/\,+ * \ ' >> /home/${USERNAME}/.bashrc && \
    echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' >> /home/${USERNAME}/.bashrc && \
    echo '       /_/ ------- \_\              ' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '-------   CCCC   H   H    OOO    I   RRRR    BBBB     OOO   TTTTT   --- ' >> /home/${USERNAME}/.bashrc && \
    echo '------  C       H   H   O   O   I   R   R   B   B   O   O    T    ----- ' >> /home/${USERNAME}/.bashrc && \
    echo '-----  C       HHHHH   O   O   I   RRRR    BBBB    O   O    T   ------ ' >> /home/${USERNAME}/.bashrc && \
    echo '----   C      H   H   O   O   I   R  R    B   B   O   O    T   ------- ' >> /home/${USERNAME}/.bashrc && \
    echo '---     CCC  H   H    OOO    I   R    R  BBBB     OOO     T  --------- ' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo 'TF' >> /home/${USERNAME}/.bashrc
##########################################################

WORKDIR /home/${USERNAME}/${WS_NAME}
ENTRYPOINT /home/${USERNAME}/final_setup.sh && /bin/bash
