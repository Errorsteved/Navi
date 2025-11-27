#!/bin/bash


IMAGE_NAME="motion1:latest"
CONTAINER_NAME="gpsimu_con1"
HOST_WS_DIR="$(pwd)/ros2_ws"
CONTAINER_WS_DIR="/ros2_ws"
echo "IMAGE_NAME=${IMAGE_NAME}"


if [ ! -d "${HOST_WS_DIR}" ]; then
  echo "未找到工作区目录: ${HOST_WS_DIR}"
  exit 1
fi

docker run -it \
  --name ${CONTAINER_NAME} \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/errorsteved/dev:/hostshare/dev" \
  --device=/dev/ttyUSB1 \
  --device=/dev/ttyUSB2 \
  ${IMAGE_NAME} bash