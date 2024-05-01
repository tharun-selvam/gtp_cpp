cmake_minimum_required(VERSION 3.14)
project(gtpcpp)

set(CMAKE_CXX_STANDARD 14)
set(EIGEN_ROOT_DIR "/usr/local/include/eigen/3.4.0_1/include/eigen3/")
set(GUROBI_ROOT_DIR "/Library/gurobi1101/macos_universal2/include")
set(GUROBI_LIB_DIR "/Library/gurobi1101/macos_universal2/lib")

include_directories(
    ${EIGEN_ROOT_DIR}
    ${GUROBI_ROOT_DIR}
)

add_executable(test test.cpp)
add_executable(track track.cpp)
add_executable(optim optim.cpp)

link_directories(${GUROBI_LIB_DIR})

target_link_libraries(optim ${GUROBI_LIB_DIR}/libgurobi_c++.a ${GUROBI_LIB_DIR}/libgurobi110.dylib)