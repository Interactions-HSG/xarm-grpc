cmake_minimum_required(VERSION 3.10)

# set the project name
project(xarm-commander)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(commander_src
    src/xarm-commander.cpp
)

add_executable(xarm-commander src/xarm-commander.cpp)

target_include_directories(xarm-CPLUS-SDK
    PRIVATE
        lib/
        
