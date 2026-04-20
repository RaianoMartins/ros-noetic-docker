FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros-noetic
ARG USER_UID=1000
ARG USER_GID=1000

# ======================================
# Install system packages
# ======================================
RUN apt-get update && apt-get install -y \
    ros-noetic-nmea-msgs \
    ros-noetic-nmea-comms \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    ros-noetic-usb-cam \
    git \
    nano \
    minicom \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    sudo \
    iproute2 \
    iputils-ping \
    net-tools \
    dnsutils \
    ffmpeg \
    libusb-1.0-0-dev \
    libsdl2-dev \
    build-essential \
    cmake \
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
# Install DJI Onboard SDK 4.1.0
# ======================================
WORKDIR /opt
RUN git clone -b 4.1.0 https://github.com/dji-sdk/Onboard-SDK.git

WORKDIR /opt/Onboard-SDK/build
RUN cmake .. && make -j$(nproc) && make install

ENV OSDK_PATH=/opt/Onboard-SDK
#ENV LD_LIBRARY_PATH=/opt/Onboard-SDK/build:${LD_LIBRARY_PATH}
#ENV CMAKE_PREFIX_PATH=/opt/Onboard-SDK:${CMAKE_PREFIX_PATH}

ENV LD_LIBRARY_PATH=/opt/Onboard-SDK/build
ENV CMAKE_PREFIX_PATH=/opt/Onboard-SDK

# ======================================
# Switch to non-root user
# ======================================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ======================================
# Create ROS workspace as the user
# ======================================
RUN mkdir -p /home/${USERNAME}/catkin_ws/src

# ======================================
# Clone OSDK ROS 4.1.0 
# ======================================
RUN git clone -b 4.1.0 https://github.com/dji-sdk/Onboard-SDK-ROS.git \
    /home/${USERNAME}/catkin_ws/src/Onboard-SDK-ROS

# Build initial empty workspace
RUN bash -c "source /opt/ros/noetic/setup.bash && \
    cd /home/${USERNAME}/catkin_ws && \
    catkin_make -j$(nproc)"
    
# ======================================
# Copy configuration files
# ======================================
COPY --chown=${USERNAME}:${USERNAME} entrypoint.sh /entrypoint.sh
COPY --chown=${USERNAME}:${USERNAME} bashrc /home/${USERNAME}/.bashrc
COPY --chown=${USERNAME}:${USERNAME} ssh-check.sh /home/${USERNAME}/ssh-check.sh

RUN chmod +x /entrypoint.sh /home/${USERNAME}/ssh-check.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export OSDK_PATH=/opt/Onboard-SDK" >> /home/${USERNAME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/Onboard-SDK/build:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bashrc
    
# ======================================
# Entrypoint
# ======================================
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
