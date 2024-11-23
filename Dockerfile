# Base image
FROM osrf/ros:noetic-desktop-full-focal

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  gazebo11 \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-controller \
  ros-noetic-robot-state-publisher \
  ros-noetic-robot-localization \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-tools \
  git \
  nano \
  dos2unix \
  python \
  && rm -rf /var/lib/apt/lists/*

# make workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy the files in the current directory into the container
COPY /tortoisebot_gazebo /catkin_ws/src/tortoisebot_gazebo
COPY /tortoisebot_description /catkin_ws/src/tortoisebot_description
COPY /tortoisebot_waypoints /catkin_ws/src/tortoisebot_waypoints

# Preparing files
Run chmod +x /catkin_ws/src/tortoisebot_waypoints/src/tortoisebot_waypoints/tortoisebot_action_server.py \
    && chmod +x /catkin_ws/src/tortoisebot_waypoints/test/test_tortoise_action.py \
    && chmod +x /catkin_ws/src/tortoisebot_waypoints/test/waypoints_test.test \
    && dos2unix /catkin_ws/src/tortoisebot_waypoints/src/tortoisebot_waypoints/tortoisebot_action_server.py \
    && dos2unix /catkin_ws/src/tortoisebot_waypoints/test/test_tortoise_action.py

# Source ros noetic and build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make"

# Source the workspace on container start
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Starting a bash shell
CMD ["bash"]