# ------ configurable prefixes for use in China (set with --build-arg) ------
# Option A: set this and build normally:
#   docker build -t orbslam3:opencv4 --build-arg REGISTRY=docker.m.daocloud.io .
# Option B: pre-pull and retag, then build with defaults (works too)
ARG REGISTRY=docker.m.daocloud.io
FROM ${REGISTRY}/library/ubuntu:22.04

# GitHub proxy prefix (leave empty to clone github.com directly):
#   --build-arg GH_PROXY=https://gh-proxy.com/
ARG GH_PROXY=https://gh-proxy.com/

# apt mirror to use inside the image (use empty string to keep default repos)
ARG APT_MIRROR=http://mirrors.ustc.edu.cn

ENV DEBIAN_FRONTEND=noninteractive

# Swap apt sources to a China mirror before the first update
RUN if [ -n "$APT_MIRROR" ]; then \
        sed -i "s|http://archive.ubuntu.com|${APT_MIRROR}|g; s|http://security.ubuntu.com|${APT_MIRROR}|g" /etc/apt/sources.list; \
    fi

# ---------- 1. Build tools + ORB-SLAM3 + Pangolin dependencies ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libopencv-dev libeigen3-dev libboost-dev libboost-serialization-dev \
    libssl-dev libgoogle-glog-dev libunwind-dev \
    python3-pip libgl1-mesa-dev libglew-dev libepoxy-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    && rm -rf /var/lib/apt/lists/*

# ---------- 2. Pangolin (pinned to v0.8) ----------
WORKDIR /opt
RUN git clone --depth 1 --branch v0.8 ${GH_PROXY}https://github.com/stevenlovegrove/Pangolin.git && \
    mkdir Pangolin/build && cd Pangolin/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . -j$(nproc) && \
    make install && ldconfig && \
    cd / && rm -rf /opt/Pangolin

# ---------- 3. Clone and build ORB-SLAM3 ----------
WORKDIR /root
RUN git clone ${GH_PROXY}https://github.com/AreteQin/ORB_SLAM3_OpenCV4.git

WORKDIR /root/ORB_SLAM3_OpenCV4
RUN sed -i 's/^sudo /# sudo /' build.sh && \
    bash ./build.sh && \
    ldconfig && \
    rm -rf Thirdparty/DBoW2/build Thirdparty/g2o/build Thirdparty/Sophus/build

ENV LD_LIBRARY_PATH=/root/ORB_SLAM3_OpenCV4/lib:/root/ORB_SLAM3_OpenCV4/Thirdparty/DBoW2/lib:/root/ORB_SLAM3_OpenCV4/Thirdparty/g2o/lib

CMD ["/bin/bash"]
