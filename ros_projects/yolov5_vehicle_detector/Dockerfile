FROM nvcr.io/nvidia/l4t-tensorrt:r8.5.2.2-devel

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
    colcon-common-extensions


COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --ignore-installed -i https://pypi.tuna.tsinghua.edu.cn/simple -r /tmp/requirements.txt

# 安装 PyTorch
WORKDIR /pakdepends
RUN wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    pip install ./torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

# 安装 torchvision（含代理设置）       不要在dockerbuild时安装，在docker的bash里安装
#RUN export http_proxy="http://192.168.1.185:7890" && \
#    export https_proxy="http://192.168.1.185:7890" && \
#    git clone --branch v0.16.1 https://github.com/pytorch/vision torchvision && \
#    cd torchvision && export BUILD_VERSION=v0.16.1 && python3 setup.py install --user && \
#    unset http_proxy https_proxy

RUN mkdir -p /ros2_ws/src
COPY ros2_ws/src/ /ros2_ws/src/


COPY ros2_ws/test.sh /ros2_ws/test.sh
COPY ros2_ws/run.sh /ros2_ws/run.sh
RUN chmod +x /ros2_ws/test.sh /ros2_ws/run.sh

WORKDIR /ros2_ws


CMD ["bash"]
