FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
        libsdl2-dev \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /build_space

COPY shared ./shared
COPY ros2ws ./ros2ws

WORKDIR /build_space/ros2ws

RUN rosdep update
RUN rosdep install --from-paths src -y --ignore-src

RUN rm -rf build install log
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --merge-install"

CMD ["/bin/bash"]
