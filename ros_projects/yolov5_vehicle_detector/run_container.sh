#!/bin/bash


IMAGE_NAME="yolov5_detector4:latest"
CONTAINER_NAME="yolov5_container4"
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
  --device=/dev/video0 \
  --runtime nvidia \
  ${IMAGE_NAME} bash
