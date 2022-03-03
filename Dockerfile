# syntax=docker/dockerfile:1
ARG GCC_VERSION=11.2.0

# this stage builds cmake
FROM gcc:$GCC_VERSION AS cmake

ARG CMAKE_VERSION=3.22.0

RUN set -ex \
 && wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh \
 -q -O /tmp/cmake-install.sh \
 && chmod u+x /tmp/cmake-install.sh \
 && mkdir -p /usr/bin/cmake \
 && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
 && rm /tmp/cmake-install.sh

# this stage builds cmake
FROM gcc:$GCC_VERSION AS grpc

ARG GRPC_VERSION=1.43.0

# copy cmake from the cmake image
COPY --from=cmake /usr/bin/cmake /usr/bin/cmake
RUN chmod u+x /usr/bin/cmake/bin/cmake
ENV PATH="/usr/bin/cmake/bin:$PATH"

RUN apt-get update && apt-get install -y \
    autoconf \
    build-essential \
    libssl-dev \
    libtool \
    pkg-config \
 && rm -rf /var/lib/apt/lists/*

ENV MY_INSTALL_DIR=/app/grpc
ENV PATH="$MY_INSTALL_DIR/bin:$PATH"

RUN mkdir -p $MY_INSTALL_DIR

WORKDIR /tmp
RUN git clone --recurse-submodules -b v${GRPC_VERSION} https://github.com/grpc/grpc \
 && cd grpc \
 && mkdir -p cmake/build \
 && cd cmake/build \
 && cmake \
 -DgRPC_INSTALL=ON \
 -DgRPC_BUILD_TESTS=OFF \
 -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
 ../.. \
 && make \
 && make install

# this stage builds xarm-grpc
FROM gcc:$GCC_VERSION

# copy cmake from the cmake image
COPY --from=cmake /usr/bin/cmake /usr/bin/cmake
RUN chmod u+x /usr/bin/cmake/bin/cmake
ENV PATH="/usr/bin/cmake/bin:$PATH"

# copy grpc and protoc from the grpc image
COPY --from=grpc /app/grpc /usr/local/lib/grpc
RUN chmod u+x /usr/local/lib/grpc/bin/protoc
ENV PATH="/usr/local/lib/grpc/bin:$PATH"

# copy xarm-grpc src
COPY . /app

WORKDIR /app
# install the dependencies for xarm-grpc
RUN make -C libs/xArm-CPLUS-SDK xarm \
 && make install -C libs/xArm-CPLUS-SDK

# build xarm-grpc
RUN mkdir -p cmake/build \
 && cd cmake/build \
 && cmake -DCMAKE_PREFIX_PATH=/usr/local/lib/grpc ../.. \
 && make -j

EXPOSE 50051

ENTRYPOINT ["/app/cmake/build/xarm-grpc-service"]
CMD ["--help"]
