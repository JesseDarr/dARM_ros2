##################### Base ###################################################
FROM ubuntu:24.04
ARG  FLAVOR=sim

# Environmental Variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=America/Chicago
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Abort on non 0 return code 
RUN set -e

# Install Basics
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        locales \
        sudo \
        tzdata \
        git \
        build-essential \
        python3-pip \
        joystick

# Clean APT cache
RUN rm -rf /var/lib/apt/lists/*

# Set Locale
RUN locale-gen en_US.UTF-8

# ROS2 and Gzebo repo keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 repo
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS2 full desktop version (includes rviz, xacro, etc)
RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-desktop
RUN apt-get install -y ros-${ROS_DISTRO}-xacro
RUN apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-control
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-controllers
RUN apt-get install -y ros-${ROS_DISTRO}-controller-manager
RUN apt-get install -y ros-${ROS_DISTRO}-can-msgs
RUN apt-get install -y python3-colcon-common-extensions

##################### Flavor Specific ######################################## 
# sim
RUN if [ "$FLAVOR" = "sim" ]; then \
        apt-get update && \
        apt-get install -y ros-${ROS_DISTRO}-ros-gz-sim && \
        apt-get install -y ros-${ROS_DISTRO}-ros-gz-bridge && \
        apt-get install -y ros-${ROS_DISTRO}-gz-ros2-control && \
        apt-get install -y mesa-utils ; \
    fi

# robot
RUN if [ "$FLAVOR" = "robot" ]; then \
        apt-get update && \
        apt-get install -y can-utils && \
        apt-get install -y python3-can && \
        apt-get install -y ros-${ROS_DISTRO}-hardware-interface && \
        apt-get install -y ros-${ROS_DISTRO}-pluginlib ; \
        rm -rf /var/lib/apt/lists/* ; \
    fi

##################### Build ##################################################
# Create workspace
RUN mkdir -p /ros_ws/src

# Clone dependencies directly
RUN if [ "$FLAVOR" = "robot" ]; then \
        cd /ros_ws/src && \
        git clone https://github.com/odriverobotics/ros_odrive.git ; \
    fi

# Set workdir and copy files into image
WORKDIR /ros_ws/src/dARM_ros2
COPY . .

# Make scripts executable
RUN chmod +x bringup/*.sh

# Build everything
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --symlink-install && \
                  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc && \
                  echo 'source /ros_ws/install/setup.bash' >> /etc/bash.bashrc"

##################### Runtime ################################################ 
# Set XDG_RUNTIME_DIR to avoid warnings
ENV XDG_RUNTIME_DIR=/tmp/runtime-${USER}
RUN mkdir -p ${XDG_RUNTIME_DIR} && chmod 700 ${XDG_RUNTIME_DIR}

# Final workdir & default shell
WORKDIR /ros_ws/src/dARM_ros2
CMD ["/bin/bash"]