cmake_minimum_required(VERSION 3.10)
project(kinova_valve_opening)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
        #        INCLUDE_DIRS include
        #        LIBRARIES
        #        CATKIN_DEPENDS
)
catkin_python_setup()

catkin_install_python(PROGRAMS
        scripts/mock_modules.py
        scripts/piloting_demo.pyas
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})