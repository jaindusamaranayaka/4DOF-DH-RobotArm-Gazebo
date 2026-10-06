FROM osrf/ros:humble-desktop

# Install Gazebo, ROS 2 control bridges, and build tools
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create the workspace and copy your URDF/code inside
WORKDIR /ros2_ws
COPY src /ros2_ws/src

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the setup files automatically on terminal startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
