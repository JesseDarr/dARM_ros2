### Build (ROS 2 Jazzy, Ubuntu 24.04)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this package (or pull main robot repo which includes it as a sub-module)
git clone --recursive https://github.com/JesseDarr/dARM_ros2.git

# Pull third-party dependencies
vcs import ~/ros2_ws/src < dARM_ros2/dependencies.repos

# Install system + ROS deps
sudo apt update
sudo apt install -y build-essential git cmake libsocketcan2 libsocketcan-dev
rosdep update
rosdep install --from-paths ~/ros2_ws/src --ignore-src -r -y

# Build everything
cd ~/ros2_ws
colcon build --symlink-install
