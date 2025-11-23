FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros-noetic 
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install system packages
RUN apt-get update && apt-get install -y \
    ros-noetic-nmea-msgs \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    terminator \
    git \
    python3-pip \
    minicom \  
    nano \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/* 

# Create non-root user and add to groups
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && usermod -aG dialout ${USERNAME} \
    && usermod -aG sudo ${USERNAME}

# Configure sudo without password
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Create SSH directory structure
RUN mkdir -p /home/${USERNAME}/.ssh \
    && chmod 700 /home/${USERNAME}/.ssh \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh

# Pre-populate known hosts
RUN ssh-keyscan gitlab.com github.com >> /home/${USERNAME}/.ssh/known_hosts 2>/dev/null \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/known_hosts \
    && chmod 644 /home/${USERNAME}/.ssh/known_hosts

# Create and build ROS workspace as root first
RUN mkdir -p /home/${USERNAME}/catkin_ws/src \
    && cd /home/${USERNAME}/catkin_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Fix permissions for the user
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Switch to user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Copy configuration files
COPY --chown=${USERNAME}:${USERNAME} entrypoint.sh /entrypoint.sh
COPY --chown=${USERNAME}:${USERNAME} bashrc /home/${USERNAME}/.bashrc
COPY --chown=${USERNAME}:${USERNAME} ssh-check.sh /home/${USERNAME}/ssh-check.sh

# Set execute permissions
RUN sudo chmod +x /entrypoint.sh /home/${USERNAME}/ssh-check.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
