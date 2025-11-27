#!/bin/bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="motion1:latest"


sudo docker build -t ${IMAGE_NAME} "${SCRIPT_DIR}"


if [ $? -eq 0 ]; then
    echo "Docker 镜像构建成功: ${IMAGE_NAME}"
else
    echo "Docker 镜像构建失败"
    exit 1
fi
