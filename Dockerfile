FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros-noetic
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# ======================================
# Install system packages
# ======================================
RUN apt-get update && apt-get install -y \
    ros-noetic-nmea-msgs \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    ros-noetic-usb-cam \
    terminator \
    git \
    nano \
    minicom \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ======================================
# Create non-root user
# ======================================
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && usermod -aG dialout ${USERNAME} \
    && usermod -aG sudo ${USERNAME} \
    && usermod -aG video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ======================================
# SSH setup
# ======================================
RUN mkdir -p /home/${USERNAME}/.ssh && \
    chmod 700 /home/${USERNAME}/.ssh && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh

RUN ssh-keyscan gitlab.com github.com >> /home/${USERNAME}/.ssh/known_hosts 2>/dev/null || true && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.ssh/known_hosts

# ======================================
# Switch to non-root user
# ======================================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ======================================
# Create ROS workspace as the user
# ======================================
RUN mkdir -p /home/${USERNAME}/catkin_ws/src

# Build initial empty workspace
RUN bash -c "source /opt/ros/noetic/setup.bash && \
    cd /home/${USERNAME}/catkin_ws && \
    catkin_make"

# ======================================
# Copy configuration files
# ======================================
COPY --chown=${USERNAME}:${USERNAME} entrypoint.sh /entrypoint.sh
COPY --chown=${USERNAME}:${USERNAME} bashrc /home/${USERNAME}/.bashrc
COPY --chown=${USERNAME}:${USERNAME} ssh-check.sh /home/${USERNAME}/ssh-check.sh

RUN chmod +x /entrypoint.sh /home/${USERNAME}/ssh-check.sh

# ======================================
# Entrypoint
# ======================================
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
