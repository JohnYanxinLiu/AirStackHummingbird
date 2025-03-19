# ========= ARGS AND INIT ===============
# either ubuntu:22.04 or l4t. ubuntu:22.04 is default
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-nvidia/cuda:12.6.3-runtime-ubuntu22.04}

ARG REAL_ROBOT=false

# ========= SETUP ===============

# from https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# ========================
# Install dev tools and common dependencies
RUN apt update && apt install -y \
    vim nano emacs wget curl tree \
    cmake build-essential \
    less htop jq \
    python3-pip \
    tmux \
    gdb \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    iputils-ping \
    net-tools \
    zstd \
    && rm -rf /var/lib/apt/lists/*

# OpenVDB used by RayFronts and VDB Mapping visualization
# Override install newer openvdb 9.1.0 for compatibility with Ubuntu 22.04  https://bugs.launchpad.net/bugs/1970108
RUN apt remove -y libopenvdb*; \
    git clone --recurse --branch v9.1.0 https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
    mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
    cmake .. && \
    make -j8 && make install && \
    cd ..; rm -rf /opt/openvdb/build

# ========== INSTALL ROS2 ==============
# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update -y && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# ENV DEBIAN_FRONTEND=

# Install any additional ROS2 packages
RUN apt update -y && apt install -y \
    ros-dev-tools \
    ros-humble-mavros \ 
    ros-humble-tf2* \
    ros-humble-stereo-image-proc \
    ros-humble-image-view \
    ros-humble-topic-tools \
    ros-humble-grid-map \
    ros-humble-domain-bridge \
    libcgal-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# ========== INSTALL PYTHON DEPENDENCIES ==========
# Install Python dependencies
RUN pip3 install \
    empy \
    future \
    lxml \
    matplotlib==3.8.4 \
    numpy==1.24.0 \
    pkgconfig \
    psutil \
    pygments \
    wheel \
    pymavlink \
    pyyaml \
    requests \
    setuptools \
    colcon-common-extensions \
    six \
    toml \
    scipy \
    torch \
    torchvision \
    pypose \
    rich \
    tqdm \
    pillow \ 
    flow_vis \
    h5py \
    evo \
    tabulate \
    einops \
    timm==0.9.12 \
    rerun-sdk==0.17 \
    yacs \
    wandb \
    && rm -rf /var/lib/apt/lists/*

# ========= ZEDX =========
WORKDIR /tmp/

RUN if [ "$REAL_ROBOT" = "true" ]; then \
    wget -O zed_sdk.zstd.run https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Ubuntu22_cuda12.1_v4.2.5.zstd.run ;\
  else \
    wget -O zed_sdk.zstd.run https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run ;\
  fi

RUN chmod +x zed_sdk.zstd.run && \
    DEBIAN_FRONTEND="noninteractive" ./zed_sdk.zstd.run -- silent runtime_only skip_od_module skip_cuda

# ========= MACVO =========

# Downloading model weights for MACVO
WORKDIR /root/model_weights
RUN wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth" && \ 
    mv /root/model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth /root/model_weights/MACVO_FrontendCov.pth && \
    rm -rf /root/model_weights/github.com

# Fixes for MACVO Integration
RUN pip install huggingface_hub
RUN pip uninstall matplotlib -y

# ========= DESKTOP VS REAL ROBOT =========
WORKDIR /tmp/

RUN if [ "$REAL_ROBOT"  = "true" ]; then \
    # Put commands here that should run for the real robot but not the sim
    echo "REAL_ROBOT is true"; \
    apt-get update && apt-get install -y \
      libimath-dev \
      && rm -rf /var/lib/apt/lists/*; \
  else \
    # Put commands here that should be run for the sim but not the real robot
    echo "REAL_ROBOT is false" ; \
  fi

# ========= NETWORKING =========
# Add ability to SSH
RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

# ========= CLEANUP =========
# Set the working directory
WORKDIR /root/ros_ws

# Cleanup. Prevent people accidentally doing git commits as root in Docker
RUN apt purge git -y \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=