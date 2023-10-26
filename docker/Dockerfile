# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:noetic-perception-focal

ENV ROS_DISTRO=noetic

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# install tools, SW needed for GUI output and ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
        build-essential \
        git \
        python3-catkin-tools \
        python3-pip \
        nano \
        curl \
        wget \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        x11-apps \
        ros-noetic-pointcloud-to-laserscan \
        ros-noetic-slam-gmapping \
        ros-noetic-rviz \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
        libomp-dev \
        libpcl-dev \
        libeigen3-dev \
    && apt-get clean \ 
    && rm -rf /var/lib/apt/lists/*

# Source setup.bash of ROS2 humble
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# RUN rosdep update && \
#     rosdep install

# Copy the ros_entrypoint.sh script from ros:ros-base
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# Set the ros_entrypoint.sh script as the entry point
ENTRYPOINT ["/ros_entrypoint.sh"]

# Create a new workspace folder for ROS2
RUN mkdir -p /root/ros_ws/src

# Set the new workspace folder as the working directory
WORKDIR /root/ros_ws