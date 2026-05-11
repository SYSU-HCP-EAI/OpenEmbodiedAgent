# moveit/moveit:noetic-ci-testing
# Sets up a base image to use for running Continuous Integration on Travis

ARG IMAGE=noetic
FROM moveit/moveit:${IMAGE}-ci
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Switch to ros-testing
RUN echo "deb http://packages.ros.org/ros-testing/ubuntu `lsb_release -cs` main" | tee /etc/apt/sources.list.d/ros1-latest.list

# Upgrade packages to ros-testing and clean apt-cache within one RUN command
RUN apt-get -qq update && \
    apt-get -qq -y dist-upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
