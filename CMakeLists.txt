cmake_minimum_required(VERSION 3.5.1)

project(XArmCommander C CXX)

include(./common.cmake)

# Proto file
get_filename_component(hw_proto "./protos/xapi.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/xapi.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/xapi.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/xapi.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/xapi.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# hw_grpc_proto
add_library(hw_grpc_proto
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs}
  ${hw_proto_srcs}
  ${hw_proto_hdrs}
  ./libs/xArm-CPLUS-SDK/build/lib
  )
target_link_libraries(hw_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

# other libs
include_directories("./libs")

# xarm-daemon
add_executable(xarm-daemon
  xarm_daemon.cc)
target_link_libraries(xarm-daemon
  hw_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  xarm)

# xarm-commander
add_executable(xarm-commander
  libs/easyloggingpp/src/easylogging++.cc
  xarm_commander.cc
  )
target_link_libraries(xarm-commander
  hw_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

