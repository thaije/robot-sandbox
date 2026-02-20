# ARST — Autonomous Robotics Simulation Testbed
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
#
# Build: docker build -t arst .
# Run:   docker run --rm --gpus '"device=0"' -e GZ_PARTITION=0 arst

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Base + ROS 2 Jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-teleop-twist-keyboard \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Project
WORKDIR /arst
COPY . .
RUN pip3 install --no-cache-dir -e .

# Add user to render group for EGL headless rendering
RUN groupadd -f render && usermod -aG render root

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENTRYPOINT ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
CMD ["python", "-m", "scenario_runner", "--help"]
