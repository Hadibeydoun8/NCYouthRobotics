#!/bin/bash

PI_HOST=rosii@192.168.12.164
IMAGE=hadibeydoun8/ncyouthrobotics:latest
CONTAINER_NAME=ncyr_robotics_container

ssh $PI_HOST << EOF
  echo "Pulling latest image..."
  sudo docker pull $IMAGE

  echo "Stopping previous container..."
  sudo docker stop $CONTAINER_NAME || true
  sudo docker rm $CONTAINER_NAME || true

  echo "Running container detached with privileged access using multi-node launch..."
  sudo docker run -d \
    --name $CONTAINER_NAME \
    --network host \
    --privileged \
    -e ROS_IP=192.168.12.164 \
    -e ROS_DOMAIN_ID=0 \
    hadibeydoun8/ncyouthrobotics:latest \
    /bin/bash -c "source /build_space/ros2ws/install/setup.sh && \
                  ros2 launch robot_drive_interface multi_node.launch.py"
EOF
