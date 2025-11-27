FROM nvcr.io/nvidia/l4t-base:35.4.1

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai


RUN sed -i 's|http://ports.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y \
    locales \
    tzdata \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    build-essential \
    python3-pip \
    python3-dev \
    git \
    nano \
    vim \
    ffmpeg \
    libgl1-mesa-glx \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libopenblas-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libopenblas-base \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


COPY resources/ros.asc /etc/apt/trusted.gpg.d/ros.asc


RUN echo "deb [arch=arm64 signed-by=/etc/apt/trusted.gpg.d/ros.asc] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-foxy-desktop && \
    echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc


RUN python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip && \
    pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "setuptools<71" "importlib_metadata>=4.4" && \
    pip install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    empy==3.3.2 \
    catkin_pkg \
    rospkg \
    lark \
    pyros-genmsg \
    numpy==1.23.1 \
    colcon-common-extensions \
    pynput \
    pyserial


RUN mkdir -p /ros2_ws
COPY ros2_ws/ /ros2_ws/


COPY ros2_ws/test.sh /ros2_ws/test.sh
COPY ros2_ws/run.sh /ros2_ws/run.sh

WORKDIR /ros2_ws


CMD ["bash"]
