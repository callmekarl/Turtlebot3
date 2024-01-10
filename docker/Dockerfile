# ROS distribution to use
ARG ROS_DISTRO=humble
# Base image

FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# Use Cyclone DDS as middleware
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=30

# Install colcon_cd
RUN apt update
RUN apt install -y python3-colcon-common-extensions 
RUN pip install setuptools==58.2.0

# Install turtlebot2 dependencies
RUN apt-get update \
  && apt-get -y install \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
  && rm -rf /var/lib/apt/lists/*

ARG ROS_DOMAIN_ID=30
ARG TURTLEBOT3_MODEL=burger

# Create Colcon workspace with external dependencies
RUN mkdir -p /Turtlebot3_ws/src
WORKDIR /Turtlebot3_ws/src
RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /Turtlebot3_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install
ENV TURTLEBOT3_MODEL=burger

# Set up the entrypoint
WORKDIR /Turtlebot3_ws
COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "/bin/bash" ]