cmake_minimum_required(VERSION 3.0.2)
project(sensor_tools)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PROJECT_DEPENDENCIES roscpp tf2_ros eigen_conversions)
find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_tools
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
   src/ft_sensor_utils.cpp
   src/ft_sensor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ft_sensor_node src/ft_sensor_node.cpp)
target_link_libraries(ft_sensor_node ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(ft_sensor_node ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)