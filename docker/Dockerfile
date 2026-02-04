FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Set timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ARG USER_ID=9999
ARG GROUP_ID=9999
ARG USER_NAME=user
ARG USER_HOME=/home/user
ARG PARALLEL_JOBS=4
ARG VERSION=latest

# Create or modify a non-root user based on the provided UID/GID
RUN set -eux; \
    EXISTING_USER=$(getent passwd "${USER_ID}" | cut -d: -f1 || true); \
    EXISTING_GROUP=$(getent group "${GROUP_ID}" | cut -d: -f1 || true); \
    if [ -n "${EXISTING_USER}" ]; then \
        usermod -l "${USER_NAME}" "${EXISTING_USER}"; \
        groupmod -n "${USER_NAME}" "${EXISTING_GROUP}"; \
    else \
        groupadd -g "${GROUP_ID}" "${USER_NAME}"; \
        useradd -l -m -u "${USER_ID}" -g "${GROUP_ID}" -s /bin/bash "${USER_NAME}"; \
    fi; \
    cp /etc/skel/.bashrc "${USER_HOME}/.bashrc";

# Install additional ROS packages not in ros-base
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbag2-storage-mcap && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /opt/drs

# Copy repos.yaml and import dependencies using vcs
COPY repos.yaml /opt/drs/
RUN mkdir src/ && vcs import src < repos.yaml

# Copy only package.xml first for dependency resolution (cached unless package.xml changes)
COPY src/bag_converter/package.xml /opt/drs/src/bag_converter/package.xml

# Initialize rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    apt-get update && \
    source /opt/ros/"${ROS_DISTRO}"/setup.bash && \
    colcon list --packages-up-to bag_converter | awk '{print $2}' | xargs rosdep install -iry --from-paths && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy full source for building
COPY src/bag_converter /opt/drs/src/bag_converter

# Build ROS packages
RUN source /opt/ros/"${ROS_DISTRO}"/setup.bash && \
    colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBAG_CONVERTER_VERSION="${VERSION}" --packages-up-to bag_converter

# Switch to the non-root user
USER "${USER_NAME}"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/drs/install/setup.bash && exec \"$@\"", "--"]
