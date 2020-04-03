cmake_minimum_required(VERSION 2.8.3)
project(platoon_control)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(car_accelerate SHARED src/car_accelerate.cc)
target_link_libraries(car_accelerate ${GAZEBO_LIBRARIES})
