FROM ros:jazzy

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install MoveIt 2, CycloneDDS, Mesa/OpenGL, and GUI tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    libglx-mesa0 \
    libglu1-mesa \
    sudo \
    curl \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Handle default Ubuntu 24.04 user 'ubuntu' (UID 1000) and create user 'dung'
ARG USERNAME=dung
ARG USER_UID=1000
ARG USER_GID=1000

RUN (userdel -r ubuntu 2>/dev/null || true) \
    && (groupdel ubuntu 2>/dev/null || true) \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG video,dialout $USERNAME

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME/denso_ws

# Setup bashrc inside container
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc \
    && echo "if [ -f /home/dung/denso_ws/install/setup.bash ]; then source /home/dung/denso_ws/install/setup.bash; fi" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
