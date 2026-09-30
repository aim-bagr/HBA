FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ARG GIT_SHA=unknown
LABEL org.opencontainers.image.revision=$GIT_SHA

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/opt/hba/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

# 1. Install prerequisites & configure Koide3 PPA for GTSAM
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gpg \
    ca-certificates \
    software-properties-common \
    && curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null \
    && echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list \
    && apt-get update

# 2. Install build & runtime dependencies (Zero ROS dependencies)
RUN apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libmetis-dev \
    libgtsam-notbb-dev \
    libomp-dev \
    zenity \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 2b. Install web interface dependencies
RUN pip3 install --no-cache-dir fastapi "uvicorn[standard]" websockets pydantic

# 3. Copy source tree
WORKDIR /opt/hba
COPY . /opt/hba

# 4. Build HBA standalone engine
WORKDIR /opt/hba/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_STANDALONE=ON \
    -DBUILD_WITH_ROS=OFF \
    && cmake --build . -j$(nproc) \
    && ln -s /opt/hba/build/hba_standalone /usr/local/bin/hba_standalone

EXPOSE 8081

ENTRYPOINT ["/usr/local/bin/hba_standalone"]
CMD ["--help"]
