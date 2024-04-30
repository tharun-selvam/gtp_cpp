cmake_minimum_required(VERSION 3.14)

project(gtpcpp)

set(EIGEN_ROOT_DIR "/usr/local/include/eigen/3.4.0_1/include/eigen3/")
include_directories(
    ${EIGEN_ROOT_DIR}
)

add_executable(test test.cpp)