# syntax=docker/dockerfile:1.4
ARG ROS_DISTRO=humble
ARG VARIANT=ros-core
FROM ros:${ROS_DISTRO}-${VARIANT}

ARG ROS_DISTRO
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install base tooling and ROS development dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    locales \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    rsync \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-setuptools \
    python3-dev \
    libeigen3-dev \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages separately to handle potential package availability issues
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install Python tooling and utilities
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    opencv-python-headless \
    pillow \
    psutil \
    pandas \
    black \
    isort \
    flake8 \
    pylint \
    pytest \
    mypy \
    pre-commit

# Prepare rosdep
RUN rosdep init || true

# Set up workspace directory
RUN mkdir -p /workspaces/shadowhound

# Bash defaults for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /home/${USERNAME}/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc && \
    echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1" >> /home/${USERNAME}/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages" >> /home/${USERNAME}/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc && \
    echo "alias cbt='colcon test'" >> /home/${USERNAME}/.bashrc && \
    echo "alias cbr='colcon build --symlink-install && source install/setup.bash'" >> /home/${USERNAME}/.bashrc && \
    echo "alias source-ws='[ -f install/setup.bash ] && source install/setup.bash'" >> /home/${USERNAME}/.bashrc && \
    echo "alias rosdep-install='rosdep install --from-paths src --ignore-src -r -y'" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/shadowhound

# Default shell
SHELL ["/bin/bash", "-c"]

CMD ["sleep", "infinity"]
