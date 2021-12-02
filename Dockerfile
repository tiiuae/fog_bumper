# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

ARG UID=1000
ARG GID=1000
ARG BUILD_NUMBER
ARG COMMIT_ID
ARG GIT_VER

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    python3-bloom \
    fakeroot \
    dh-make \
    libboost-dev \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g $GID builder && \
    useradd -m -u $UID -g $GID -g builder builder && \
    usermod -aG sudo builder && \
    echo 'builder ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN mkdir -p /fog_bumper/packaging/common

COPY packaging/rosdep.yaml /fog_bumper/packaging/
COPY packaging/common/rosdep.sh /fog_bumper/packaging/common/
COPY underlay.repos /fog_bumper/

RUN /fog_bumper/packaging/common/rosdep.sh /fog_bumper

RUN chown -R builder:builder /fog_bumper

USER builder

VOLUME /fog_bumper/sources
WORKDIR /fog_bumper/sources

RUN rosdep update
