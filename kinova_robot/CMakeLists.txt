cmake_minimum_required(VERSION 3.10)
project(kinova_robot)

set(CMAKE_CXX_STANDARD 17)
set(PACKAGE_DEPENDENCIES
    roscpp
    kortex_driver
    control_toolbox
    controller_interface
    hardware_interface
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

add_library(${PROJECT_NAME} src/hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})