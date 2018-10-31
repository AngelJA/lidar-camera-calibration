FROM ros:kinetic-perception-xenial

RUN apt update && apt install -y \
    libpcap-dev \
    ros-kinetic-rqt \
    ros-kinetic-rqt-common-plugins \
    ros-kinetic-diagnostic-updater \
    ros-kinetic-rviz \
    software-properties-common \
    terminator \
    wget

# install catkin_tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt update && apt install -y \
    python-catkin-tools

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

CMD terminator