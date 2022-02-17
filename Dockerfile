FROM ubuntu:18.04 AS base

ENV CMAKE_VERSION=3.22.0
ENV GRPC_VERSION=1.43.0

RUN apt-get update; \
    apt install -y build-essential libssl-dev wget autoconf libtool pkg-config git; \
    cd /usr/src; \
    wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz; \
    tar -zxvf cmake-${CMAKE_VERSION}.tar.gz; \
    cd cmake-${CMAKE_VERSION}; \
    ./bootstrap; \
    make; \
    make install; \
    cmake --version;

FROM base AS extend-grpc

ENV MY_INSTALL_DIR=~/.local
ENV PATH="$MY_INSTALL_DIR/bin:$PATH"

RUN cp /bin/bash /bin/sh
RUN git clone --recurse-submodules -b v${GRPC_VERSION} https://github.com/grpc/grpc; \
    cd grpc; \
    mkdir -p cmake/build; \
    pushd cmake/build; \
    cmake -DgRPC_INSTALL=ON \
          -DgRPC_BUILD_TESTS=OFF \
          -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
          ../..; \
    make; \
    make install; \
    popd;

FROM extend-grpc AS xarm-grpc

COPY ./xarm_grpc_project/. /root

RUN cd /root/xarm-grpc; \
    make -C libs/xArm-CPLUS-SDK xarm; \
    make install -C libs/xArm-CPLUS-SDK; \
    mkdir -p cmake/build; cd cmake/build; \
    cmake -DCMAKE_PREFIX_PATH=~/.local ../..; \
    make;