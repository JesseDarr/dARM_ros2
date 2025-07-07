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
        python3-pip

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

# Install ROS2 and RVIZ
RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-ros-base
RUN apt-get install -y ros-${ROS_DISTRO}-rviz2
RUN apt-get install -y python3-colcon-common-extensions

##################### Flavor Specific ######################################## 
# sim
RUN if [ "$FLAVOUR" = "sim" ]; then \
        apt-get update && \
        apt-get install -y ros-${ROS_DISTRO}-ros-gz-sim && \
        apt-get install -y ros-${ROS_DISTRO}-ros-gz-gui && \
        apt-get install -y ros-${ROS_DISTRO}-ros-gz-bridge && \
        apt-get install -y mesa-utils ; \
    fi

# robot
RUN if [ "$FLAVOR" = "robot" ]; then \
        apt-get update && \
        apt-get install -y can-utils && \
        apt-get install -y bluez ; \
    fi

##################### Build ##################################################
# Create workspace, set workdir, and copy files into image
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src/dARM_ros2
COPY . .

# Build dARM_ros2 package and add setup.bash to bash.bashrc
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --symlink-install && \
                  echo 'source /ros_ws/install/setup.bash' >> /etc/bash.bashrc"

# Set default shell
CMD ["/bin/bash"]
