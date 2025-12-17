FROM ubuntu:22.04

# Re-declare ARG after FROM to use in RUN commands
ARG ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# Set timezone and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ARG USER_ID=9999
ARG GROUP_ID=9999
ARG USER_NAME=user
ARG USER_HOME=/home/user

# Create or modify a non-root user based on the provided UID/GID
RUN set -eux; \
    EXISTING_USER=$(getent passwd ${USER_ID} | cut -d: -f1 || true); \
    EXISTING_GROUP=$(getent group ${GROUP_ID} | cut -d: -f1 || true); \
    if [ -n "${EXISTING_USER}" ]; then \
        usermod -l ${USER_NAME} ${EXISTING_USER}; \
        groupmod -n ${USER_NAME} ${EXISTING_GROUP}; \
    else \
        groupadd -g ${GROUP_ID} ${USER_NAME}; \
        useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USER_NAME}; \
    fi; \
    cp /etc/skel/.bashrc ${USER_HOME}/.bashrc;

# Install dependencies
RUN apt update && \
    apt install -y --no-install-recommends \
        software-properties-common \
        tzdata \
        locales \
        curl \
        ca-certificates \
        python3 \
        python3-pip \
        libpython3.10 \
        build-essential \
        pkg-config \
        cmake && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 repository
RUN curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -Po '"tag_name": "\K.*?(?=")' > /tmp/ros_apt_version && \
    curl -sSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$(cat /tmp/ros_apt_version)/ros2-apt-source_$(cat /tmp/ros_apt_version).jammy_all.deb" -o /tmp/ros2-apt-source.deb && \
    apt update && \
    apt install -y /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb /tmp/ros_apt_version && \
    apt update && \
    apt install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-core \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-dev-tools \
        python3-rosdep && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /opt/drs

# Copy source code
COPY src/ /opt/drs/src/

# Initialize rosdep and install dependencies
RUN rosdep init && \
    rosdep update && \
    apt update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon list --packages-up-to app oxts_msgs | awk '{print $2}' | xargs rosdep install -iry --from-paths && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Build ROS packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to app oxts_msgs

# Set env var
ENV ROS_DISTRO=${ROS_DISTRO}

# Copy entrypoint
ADD docker/entrypoint.sh /
RUN chown ${USER_ID}:${GROUP_ID} /entrypoint.sh && chmod +x /entrypoint.sh

# Switch to the non-root user
USER ${USER_NAME}

ENTRYPOINT ["/entrypoint.sh"]