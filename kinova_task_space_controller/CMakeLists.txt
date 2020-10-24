cmake_minimum_required(VERSION 3.10)
project(kinova_task_space_controller)

find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

set(PROJECT_DEPENDENCIES
        robot_control
        controller_interface
        kortex_driver
        kortex_description
        hardware_interface
        pluginlib
        roscpp
        realtime_tools
        interactive_markers
        )

find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES} ${PINOCCHIO_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/task_space_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
