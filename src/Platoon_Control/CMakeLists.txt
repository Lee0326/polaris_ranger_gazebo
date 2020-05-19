cmake_minimum_required(VERSION 2.8.3)
project(platoon_control)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED
    roscpp
    std_msgs
    nav_msgs
    message_filters
)
catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)



add_executable(platoon_controller src/platoon_control.cpp)
target_link_libraries(platoon_controller
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
