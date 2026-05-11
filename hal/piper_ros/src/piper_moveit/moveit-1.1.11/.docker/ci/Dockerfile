# moveit/moveit:noetic-ci
# Sets up a base image to use for running Continuous Integration on Travis

ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
MAINTAINER Dave Coleman dave@picknik.ai

ENV TERM xterm

# Setup (temporary) ROS workspace
WORKDIR /root/ws_moveit

# Copy MoveIt sources from docker context
COPY . src/moveit

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
        # Some basic requirements
        wget git sudo \
        # Preferred build tools
        python$(test "${ROS_DISTRO}" = "noetic" && echo 3)-catkin-tools \
        clang clang-format-10 clang-tidy clang-tools \
        ccache && \
    #
    # Download MoveIt sources, so that we can fetch all necessary dependencies
    wstool init --shallow src src/moveit/.github/workflows/upstream.rosinstall && \
    git clone --depth 1 --branch master https://github.com/ros-planning/moveit_resources src/moveit_resources && \
    #
    # Download all dependencies of MoveIt
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove the source code from this container
    rm -rf src && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/* && \
    # Globally disable git security
    # https://github.blog/2022-04-12-git-security-vulnerability-announced
    git config --global --add safe.directory "*"

# Continous Integration Setting
ENV IN_DOCKER 1
