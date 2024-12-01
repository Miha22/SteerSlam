cmake_minimum_required(VERSION 3.10)
project(steerslam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -g -std=c++11")
find_package(OpenCV 2.4.3 REQUIRED)
add_subdirectory(lib/DBoW2)
add_subdirectory(lib/g2o)
add_subdirectory(lib/ORB_SLAM2_v2)
add_subdirectory(lib/pangolin05)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/lib/orb-slam2/include
  ${PROJECT_SOURCE_DIR}/lib/DBoW2
  ${PROJECT_SOURCE_DIR}/lib/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/lib/orb-slam2
)

add_library(steerslamlib SHARED
  ${PROJECT_SOURCE_DIR}/src/io/frame_reader.cpp
  ${PROJECT_SOURCE_DIR}/src/io/frame_writer.cpp
  ${PROJECT_SOURCE_DIR}/src/slam/frame_processor.cpp
)

add_executable(render src/render.cpp)
add_executable(trajectorize src/trajectorize.cpp)

add_dependencies(trajectorize steerslamlib ORB_SLAM2)
add_dependencies(render steerslamlib ORB_SLAM2)
add_dependencies(steerslamlib ORB_SLAM2)