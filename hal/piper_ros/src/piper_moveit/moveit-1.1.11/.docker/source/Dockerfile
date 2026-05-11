# syntax = docker/dockerfile:1.3

# moveit/moveit:noetic-source
# Downloads the moveit source code and install remaining debian dependencies

ARG IMAGE=noetic
FROM moveit/moveit:${IMAGE}-ci-testing
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

ENV PYTHONIOENCODING UTF-8
# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/ws_moveit/install
# Environment variable used in instructions on moveit.ros.org website for running clang-tidy
ENV CATKIN_WS $(realpath $ROS_UNDERLAY/..)
WORKDIR $ROS_UNDERLAY/..

# Copy MoveIt sources from docker context
COPY . src/moveit

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    # Fetch required upstream sources for building
    wstool init --shallow src src/moveit/.github/workflows/upstream.rosinstall && \
    git clone --depth 1 --branch master https://github.com/ros-planning/moveit_resources src/moveit_resources && \
    #
    catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out,
    # but not too much such that the Docker log gets too long (another form of timeout)
    catkin build --limit-status-rate 0.001 --no-notify && \
    ccache -s && \
    #
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh
