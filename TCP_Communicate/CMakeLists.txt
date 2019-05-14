cmake_minimum_required(VERSION 2.80)
project(TCP_Communicate)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        )
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
add_executable(TCP_Communicate main.cpp)
target_link_libraries(TCP_Communicate ${catkin_LIBRARIES}
        )

