FROM px4io/px4-dev:v1.17.0 AS dev

ARG ROS_DISTRO=jazzy
ARG UBUNTU_CODENAME=noble
ARG MICRO_XRCE_DDS_AGENT_TAG=v2.4.3
ARG PX4_MSGS_BRANCH=release/1.17

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        gnupg \
        lsb-release \
        locales \
        software-properties-common \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN ros_apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | awk -F'"' '/"tag_name"/ {print $4; exit}')" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${UBUNTU_CODENAME}_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libzstd-dev \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-dev-tools \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
        ros-${ROS_DISTRO}-ament-cmake-cppcheck \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL -o /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i /tmp/cuda-keyring.deb \
    && rm /tmp/cuda-keyring.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        cuda-compiler-12-6 \
        cuda-cudart-dev-12-6 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/px4_msgs_ws/src \
    && git clone --depth 1 --branch "${PX4_MSGS_BRANCH}" https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon --log-base /opt/px4_msgs_ws/log build \
        --base-paths /opt/px4_msgs_ws/src \
        --build-base /opt/px4_msgs_ws/build \
        --install-base /opt/px4_msgs_ws/install \
        --merge-install \
    && rm -rf /opt/px4_msgs_ws/build /opt/px4_msgs_ws/log

RUN git clone --depth 1 --branch "${MICRO_XRCE_DDS_AGENT_TAG}" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/Micro-XRCE-DDS-Agent \
    && cmake -S /opt/Micro-XRCE-DDS-Agent -B /opt/Micro-XRCE-DDS-Agent/build \
    && cmake --build /opt/Micro-XRCE-DDS-Agent/build --parallel "$(nproc)" \
    && cmake --install /opt/Micro-XRCE-DDS-Agent/build \
    && ldconfig \
    && rm -rf /opt/Micro-XRCE-DDS-Agent

RUN cat >/etc/profile.d/drone-gazebo.bash <<EOF
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/px4_msgs_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="\${GZ_SIM_RESOURCE_PATH:-}"
EOF

WORKDIR /workspace

CMD ["bash", "-l"]


# Roadmap item 27 block 2 (steps 1 to 3): the onboard image, what a flight
# computer carries and nothing of the simulator: ROS 2 base, rclcpp, the
# messages, Eigen, OpenCV, zstd, px4_msgs, and CUDA 13 on amd64 (on a Jetson
# CUDA comes with JetPack: CUDA=off for the arm64 build under emulation). The
# package is built in it from the mounted workspace with
# -DDRONE_CITY_NAV_ENABLE_SIMULATION=OFF (tools/onboard_image.sh).
FROM ros:jazzy-ros-base AS onboard

ARG ROS_DISTRO=jazzy
ARG PX4_MSGS_BRANCH=release/1.17
ARG CUDA=on

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg \
        libeigen3-dev \
        libopencv-dev \
        libzstd-dev \
        locales \
        pkg-config \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-index-cpp \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-visualization-msgs \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN if [[ "${CUDA}" == on ]]; then \
        curl -fsSL -o /tmp/cuda-keyring.deb \
            https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
        && dpkg -i /tmp/cuda-keyring.deb \
        && rm /tmp/cuda-keyring.deb \
        && apt-get update \
        && apt-get install -y --no-install-recommends \
            cuda-compiler-13-0 \
            cuda-cudart-dev-13-0 \
        && rm -rf /var/lib/apt/lists/* \
        && ln -s /usr/local/cuda-13.0 /usr/local/cuda; \
    fi
ENV PATH=/usr/local/cuda/bin:${PATH}

RUN mkdir -p /opt/px4_msgs_ws/src \
    && git clone --depth 1 --branch "${PX4_MSGS_BRANCH}" https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon --log-base /opt/px4_msgs_ws/log build \
        --base-paths /opt/px4_msgs_ws/src \
        --build-base /opt/px4_msgs_ws/build \
        --install-base /opt/px4_msgs_ws/install \
        --merge-install \
    && rm -rf /opt/px4_msgs_ws/build /opt/px4_msgs_ws/log

RUN cat >/etc/profile.d/drone-gazebo.bash <<EOF
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/px4_msgs_ws/install/setup.bash
EOF

WORKDIR /workspace

CMD ["bash", "-l"]
