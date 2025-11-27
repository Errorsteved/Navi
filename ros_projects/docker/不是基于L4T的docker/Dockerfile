FROM ros:foxy-ros-base


ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai


RUN sed -i 's|http://.*.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y \
    tzdata \
    python3.8 python3.8-dev python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    curl wget git lsb-release \
    ros-foxy-rclpy \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-builtin-interfaces \
    ros-foxy-launch \
    && rm -rf /var/lib/apt/lists/*


RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1


RUN python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip

RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    "setuptools<71" "importlib_metadata>=4.4"


RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    empy==3.3.2 \
    catkin_pkg \
    rospkg \
    lark


COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --ignore-installed -i https://pypi.tuna.tsinghua.edu.cn/simple -r /tmp/requirements.txt


WORKDIR /ros2_ws
COPY ros2_ws/src/ src/


RUN mkdir -p src


CMD ["bash"]
