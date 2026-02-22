# Use NVIDIA CUDA base image with Ubuntu 20.04 (Focal)
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# -----------------------------
# Install basic tools
# -----------------------------
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo software-properties-common \
    nano git wget build-essential cmake \
    libopencv-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Install cuDNN 8 for CUDA 11
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=8.9.2.26-1+cuda11.8 \
    libcudnn8-dev=8.9.2.26-1+cuda11.8 && \
    rm -rf /var/lib/apt/lists/*

# Install ROS Noetic
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get update && apt-get install -y ros-noetic-desktop-full \
    python3-catkin-tools python3-wstool python3-osrf-pycommon && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------
# Install ONNX Runtime GPU
# -----------------------------
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-x64-gpu-1.16.3.tgz && \
    tar -zxvf onnxruntime-linux-x64-gpu-1.16.3.tgz && \
    cp -r onnxruntime-linux-x64-gpu-1.16.3/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-x64-gpu-1.16.3/lib/* /usr/local/lib/ && \
    ldconfig && \
    rm -rf onnxruntime-linux-x64-gpu-1.16.3.tgz onnxruntime-linux-x64-gpu-1.16.3

# -----------------------------
# Install Ceres Solver 2.1
# -----------------------------
RUN git clone https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-solver && \
    cd /tmp/ceres-solver && git checkout 2.1.0 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF && \
    make -j4 && make install && \
    cd / && rm -rf /tmp/ceres-solver

# -----------------------------
# Setup catkin workspace for SuperVINS
# -----------------------------
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Source ROS and build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -j4"

# -----------------------------
# XDG runtime fix for RViz
# -----------------------------
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# -----------------------------
# Auto-source ROS + workspace
# -----------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc
