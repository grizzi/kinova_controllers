cmake_minimum_required(VERSION 3.10)
project(kinova_robot)

set(CMAKE_CXX_STANDARD 17)
set(PACKAGE_DEPENDENCIES
    roscpp
    kortex_driver
    control_toolbox
    controller_interface
    hardware_interface
    eigen_conversions
    angles)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-reorder")
add_definitions(-D_OS_UNIX)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/command_interface.cpp
  src/hardware_interface.cpp
  src/kinova_robot_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Robot Driver ROS node
add_executable(${PROJECT_NAME}_node src/kinova_robot_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})