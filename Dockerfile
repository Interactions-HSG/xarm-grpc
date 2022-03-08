# syntax=docker/dockerfile:1
ARG GCC_VERSION=11.2.0
ARG CMAKE_VERSION=3.22.0
ARG GRPC_VERSION=1.43.0

FROM iomz/cmake:$CMAKE_VERSION AS cmake
FROM iomz/grpc-protoc:$GRPC_VERSION AS grpc
FROM pseudomuto/protoc-gen-doc as protocGenDoc

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

# copy protoc gen doc from the protocGenDoc image
COPY --from=protocGenDoc /usr/bin/protoc-gen-doc /usr/bin/protoc-gen-doc
RUN chmod u+x /usr/bin/protoc-gen-doc
ENV PATH="/usr/bin/protoc-gen-doc:$PATH"

# copy xarm-grpc src
COPY . /app

WORKDIR /app
# generate documentation for the proto file
RUN mkdir -p doc \
    && protoc --doc_out=./doc --doc_opt=html,index.html protos/*.proto

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
