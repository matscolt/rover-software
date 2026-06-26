FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

WORKDIR /ros2_ws

COPY src/ src/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc
ENV AMENT_PREFIX_PATH=/ros2_ws/install/my_ros2_pkg:/opt/ros/humble
ENV PYTHONPATH=/ros2_ws/install/my_ros2_pkg/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages

CMD ["/bin/bash"]
