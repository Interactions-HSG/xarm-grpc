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
