#####################  Base  ###################################################
FROM ubuntu:24.04
ARG  FLAVOUR=sim               # sim  |  robot

# ── Environment ───────────────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND    noninteractive
ENV LANG               C.UTF-8
ENV LC_ALL             C.UTF-8
ENV TZ                 America/Chicago          # CST/CDT
ENV ROS_DISTRO         jazzy
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp         # DDS middleware for ROS 2

# ── Essentials ─ one package per line for clarity ────────────────────────────
RUN apt-get update
RUN apt-get install -y curl
RUN apt-get install -y gnupg
RUN apt-get install -y lsb-release
RUN apt-get install -y locales
RUN apt-get install -y sudo
RUN apt-get install -y nano
RUN apt-get install -y tzdata
RUN apt-get install -y git
RUN apt-get install -y build-essential
RUN apt-get install -y python3-pip

# ── Locale ────────────────────────────────────────────────────────────────────
RUN locale-gen en_US.UTF-8

# ── ROS 2 repo key (download → convert ASCII → binary keyring) ────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# ── Add ROS 2 APT repo line for this Ubuntu (jammy/noble/etc.) ────────────────
RUN echo "deb [arch=$(dpkg --print-architecture) \
          signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/ros2.list

# ── ROS 2 base + RViz2 ────────────────────────────────────────────────────────
RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-ros-base
RUN apt-get install -y ros-${ROS_DISTRO}-rviz2   # visualiser

#####################  Flavour-specific  #######################################
# Simulation extras
RUN if [ "$FLAVOUR" = "sim" ]; then \
        apt-get update && \
        apt-get install -y gazebo && \
        apt-get install -y mesa-utils ; \
    fi

# Robot-only extras
RUN if [ "$FLAVOUR" = "robot" ]; then \
        apt-get update && \
        apt-get install -y can-utils && \
        apt-get install -y bluez ; \
    fi

#####################  Workspace build  #######################################
# ── Create canonical colcon layout ────────────────────────────────────────────
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src/dARM_ros2       # repo lives here
COPY . /ros_ws/src/dARM_ros2

# ── Build & make every future shell auto-source the workspace ────────────────
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --symlink-install && \
                  echo 'source /ros_ws/install/setup.bash' >> /etc/bash.bashrc"

#####################  Default shell  ##########################################
CMD ["/bin/bash"]
