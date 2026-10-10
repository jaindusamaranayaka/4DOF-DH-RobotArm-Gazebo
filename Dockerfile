FROM osrf/ros:humble-desktop

# Install Gazebo, ROS 2 control bridges, and build tools
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-joint-state-publisher-gui \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create the workspace and copy your URDF/code inside
WORKDIR /ros2_ws
COPY src /ros2_ws/src

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the setup files automatically on terminal startup and set Gazebo resource paths
ENV IGN_GAZEBO_RESOURCE_PATH=/ros2_ws/install/gp7_description/share:${IGN_GAZEBO_RESOURCE_PATH}
ENV GZ_SIM_RESOURCE_PATH=/ros2_ws/install/gp7_description/share:${GZ_SIM_RESOURCE_PATH}

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export IGN_GAZEBO_RESOURCE_PATH=/ros2_ws/install/gp7_description/share:\$IGN_GAZEBO_RESOURCE_PATH" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/gp7_description/share:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc
