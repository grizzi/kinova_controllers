cmake_minimum_required(VERSION 3.10)
project(kinova_sensors)

find_package(catkin REQUIRED COMPONENTS )

catkin_package()

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
