FROM nvidia/cuda:10.1-cudnn7-devel

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y \
    python3-opencv ca-certificates python3-dev git wget sudo jupyter ffmpeg python3-empy vim \
    cmake ninja-build protobuf-compiler libprotobuf-dev python3-pip python-pip

## create a non-root user
ARG USER_ID=1000
RUN useradd -m --no-log-init --system  --uid ${USER_ID} arg -g sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER arg
WORKDIR /home/arg

ENV PATH="/home/arg/.local/bin:${PATH}"
RUN wget https://bootstrap.pypa.io/get-pip.py && \
    python3 get-pip.py --user && \
    rm get-pip.py

## PIP3 Install
RUN pip3 install gdown rospkg catkin_pkg 

## install dependencies
## See https://pytorch.org/ for other options if you use a different version of CUDA
RUN pip3 install --user tensorboard cython youtube-dl
RUN pip3 install --user torch==1.5+cu101 torchvision==0.6+cu101 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install --user 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'

RUN pip3 install --user 'git+https://github.com/facebookresearch/fvcore'
## install detectron2
RUN git clone https://github.com/facebookresearch/detectron2 detectron2
## set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
## This will by default build detectron2 for all common cuda architectures and take a lot more time,
## because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

RUN pip3 install --user -e /home/arg/detectron2
RUN python3 -m pip install -e /home/arg/detectron2


## Set a fixed model cache directory.
# ENV FVCORE_CACHE="/tmp"

## install packages
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sudo apt-get -o Acquire::ForceIPv4=true update && sudo apt-get -o Acquire::ForceIPv4=true install -q -y \
    net-tools \
    ros-melodic-desktop \
    ros-melodic-perception \
    ros-melodic-rgbd-launch \
    ros-melodic-desktop-full \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-cartographer-ros \
    ros-melodic-cartographer-rviz \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-geodesy \
    ros-melodic-serial \
    ros-melodic-controller-manager \
    ros-melodic-openslam-gmapping \
    ros-melodic-twist-mux \
    ros-melodic-velodyne-simulator \
    ros-melodic-gazebo-ros \
    ros-melodic-lms1xx \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-teleop-twist-joy \
    ros-melodic-ddynamic-reconfigure \
    ros-melodic-rosserial-arduino \
    ros-melodic-rosserial 

######################################## realsense ########################################

RUN sudo apt-get update && sudo apt-get install -y \
    libsuitesparse-dev\
    software-properties-common \
    libusb-1.0-0-dev \
    libx11-dev \
    xorg-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev 
    
# set the version of the realsense library
ENV LIBREALSENSE_VERSION 2.24.0
RUN wget https://github.com/IntelRealSense/librealsense/archive/v${LIBREALSENSE_VERSION}.tar.gz && \
    tar -xvzf v${LIBREALSENSE_VERSION}.tar.gz && \
    rm v${LIBREALSENSE_VERSION}.tar.gz && \
    mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
    cd librealsense-${LIBREALSENSE_VERSION}/build && \
    cmake .. && \
    sudo make && \
    sudo make install && \
    sudo rm -rf librealsense-${LIBREALSENSE_VERSION}