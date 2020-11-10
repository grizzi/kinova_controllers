cmake_minimum_required(VERSION 3.10)
project(kinova_sim)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
        #        INCLUDE_DIRS include
        #        LIBRARIES
        #        CATKIN_DEPENDS
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})