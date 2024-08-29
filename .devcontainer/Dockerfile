FROM osrf/ros:humble-desktop
ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV DEBIAN_FRONTEND=dialog
ENV SHELL /bin/bash

RUN apt-get update && \
    sudo apt-get install -y lsb-release wget gnupg

ENV DEBIAN_FRONTEND=noninteractive
RUN sudo apt-get update --fix-missing \
    && apt-get -y install bash-completion ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-generate-parameter-library \
    python3-pip pre-commit \
    && pip3 uninstall numpy \
    && pip3 install numpy==1.26.4 \
    && pip3 install mediapipe \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ARG WORKSPACE
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc
