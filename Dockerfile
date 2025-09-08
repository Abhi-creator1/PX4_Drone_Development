# PX4 v1.15 + ROS2 Humble + Rust + Micro XRCE-DDS Agent (using system Fast DDS)
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin
ENV ROS_DISTRO=humble
WORKDIR /workspace

# Essential dependencies
RUN apt-get update && apt-get install -y \
    curl wget git sudo \
    build-essential cmake pkg-config \
    python3 python3-pip python3-dev \
    gnupg lsb-release \
    libasio-dev libtinyxml2-dev \
    libfoonathan-memory-dev \
    libfastcdr-dev libfastrtps-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 setup
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b v1.15.0
WORKDIR /workspace/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx
RUN make px4_sitl

# ROS2 setup
WORKDIR /workspace
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu jammy main" \
      | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-humble-desktop python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --user empy==3.3.4 pyros-genmsg setuptools
RUN rosdep init && rosdep update

# Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"


# ROS2 workspace with PX4 messages
WORKDIR /workspace
RUN mkdir -p ros2_ws/src
WORKDIR /workspace/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git -b release/1.15
RUN git clone https://github.com/PX4/px4_ros_com.git
WORKDIR /workspace/ros2_ws
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Scripts
WORKDIR /workspace
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /workspace/ros2_ws/install/setup.bash\nexport GAZEBO_MODEL_PATH=/workspace/PX4-Autopilot/Tools/simulation/gz/models:$GAZEBO_MODEL_PATH' \
      > setup_env.sh && chmod +x setup_env.sh
RUN echo '#!/bin/bash\ncd /workspace/PX4-Autopilot\nmake px4_sitl gz_x500' \
      > start_px4.sh && chmod +x start_px4.sh


WORKDIR /workspace
CMD ["/bin/bash"]
