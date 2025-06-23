FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Avoid prompts with apt-get
ARG DEBIAN_FRONTEND=noninteractive

###################################
###         Install ROS         ###
###################################

# Update system locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# 
RUN apt install software-properties-common -y && \
    add-apt-repository universe
    
# Add the ROS 2 GPG key with apt.
RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt upgrade -y && \
    apt install ros-humble-desktop -y

# Install pip3 
RUN apt-get update && apt-get install -y python3-pip

#####################################
###         Install PyTorch       ###
#####################################

# Install OpenBLAS and other dependencies for PyTorch
RUN apt-get update && apt-get install -y \
    libopenblas-base \
    libopenblas-dev \
    libopenblas0-pthread \
    libomp-dev && \
    ln -sf /usr/lib/aarch64-linux-gnu/libopenblas.so /usr/lib/aarch64-linux-gnu/libopenblas.so.0

ARG PYTORCH_URL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
ARG PYTORCH_WHL=torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install --no-cache-dir --verbose ${PYTORCH_WHL} && \
    rm ${PYTORCH_WHL}

# Fix NumPy compatibility issue with PyTorch - install specific compatible version
RUN pip3 install --force-reinstall numpy==1.24.3

###################################
###     Install cuSPARSELt      ###
###################################

ARG cuSPARSELt_URL=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
ARG cuSPARSELt_DEB=cuda-keyring_1.1-1_all.deb

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${cuSPARSELt_URL} -O ${cuSPARSELt_DEB} && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm ${cuSPARSELt_DEB} && \
    apt-get update && \
    apt-get -y install libcusparselt0 libcusparselt-dev

###################################
###         pip packages         ###
###################################

# Install pip and Python packages
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install setuptools Cython wheel numpy>=1.21.0 colcon-common-extensions canopen Jetson.GPIO

RUN apt-get install busybox && apt-get install iproute2 -y && \
    apt-get install net-tools -y


#####################################
###         Install EXTRAS       ###
#####################################


#####################################
###       Entrypoint script       ###
#####################################

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
RUN rm -rf /var/lib/apt/lists/*
