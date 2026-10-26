# ROS Noetic (Ubuntu 20.04) со всеми зависимостями лабораторных и собранным workspace.
# Сборка (из корня репозитория):  docker build -f docker/Dockerfile -t dp_labs .
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-gazebo-ros-control \
        ros-noetic-rqt-publisher \
        ros-noetic-rqt-plot \
        ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

# Gazebo не будет пытаться скачивать модели из интернета при старте
# (ground_plane и sun есть в локальной установке Gazebo)
ENV GAZEBO_MODEL_DATABASE_URI=""

COPY . /catkin_ws/src/DP_Labs
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
