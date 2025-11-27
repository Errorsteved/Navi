FROM nvcr.io/nvidia/l4t-tensorrt:r8.5.2-runtime

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
    && apt-get clean && rm -rf /var/lib/apt/lists/*


COPY resources/ros.asc /etc/apt/trusted.gpg.d/ros.asc


RUN echo "deb [arch=arm64 signed-by=/etc/apt/trusted.gpg.d/ros.asc] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-foxy-desktop && \
    echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc


RUN python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip

RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    "setuptools<71" "importlib_metadata>=4.4"
  

RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    empy==3.3.2 \
    catkin_pkg \
    rospkg \
    lark \
    pyros-genmsg \
    numpy \
    colcon-common-extensions


COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --ignore-installed -i https://pypi.tuna.tsinghua.edu.cn/simple -r /tmp/requirements.txt

RUN mkdir -p /ros2_ws/src
COPY ros2_ws/src/ /ros2_ws/src/


CMD ["bash"]
