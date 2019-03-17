FROM ros:melodic

RUN apt update && apt install -y \
    dbus-x11 \
    libpcap-dev \
    python-scipy \
    ros-melodic-diagnostic-updater \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-pcl-conversions \
    ros-melodic-pcl-ros \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    ros-melodic-rviz \
    ros-melodic-usb-cam \
    ros-melodic-velodyne-driver \
    ros-melodic-velodyne-simulator \
    software-properties-common \
    terminator

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY ./models /root/.gazebo/models

CMD terminator