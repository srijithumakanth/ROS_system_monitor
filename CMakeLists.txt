cmake_minimum_required(VERSION 2.6)
project(ros_system_monitor)

set(CMAKE_BUILD_TYPE Debug)

find_package(Curses REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(include
                    ${CURSES_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(ros_system_monitor ${SOURCES})

set_property(TARGET ros_system_monitor PROPERTY CXX_STANDARD 17)

target_link_libraries(ros_system_monitor 
                    stdc++fs
                    ${CURSES_LIBRARIES}
                    ${catkin_LIBRARIES})

# TODO: Run -Werror in CI.
target_compile_options(ros_system_monitor PRIVATE -Wall -Wextra)
