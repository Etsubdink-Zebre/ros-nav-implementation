# Hospital Delivery Robot — Docker with GPU
# Build:  docker build -t hospital-robot .
# Run:    docker run -it --privileged --gpus all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix hospital-robot

FROM osrf/ros:jazzy-desktop-full

# Gazebo Harmonic
RUN apt-get update && apt-get install -y --no-install-recommends -o Acquire::Retries=5 -o Acquire::ForceIPv4=true \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-tools \
    python3-pip \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN pip3 install --break-system-packages scipy numpy

# User setup
ARG USER=robot
RUN useradd -m -s /bin/bash $USER && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER
WORKDIR /home/$USER

# Environment
ENV TURTLEBOT3_MODEL=waffle
ENV DISPLAY=:0
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Copy workspace
COPY --chown=$USER hospital_ws/ ./hospital_ws/

# Build
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd ~/hospital_ws && colcon build --symlink-install"

COPY --chown=$USER entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
