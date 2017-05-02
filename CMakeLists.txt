cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(ardupilot_sitl_gazebo)


#######################
## Find Dependencies ##
#######################

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
    message(FATAL_ERROR "You need at least Gazebo 7.0. Your version: ${GAZEBO_VERSION}")
else()
    message("Gazebo version: ${GAZEBO_VERSION}")
endif()

###########
## Build ##
###########

include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${GAZEBO_INCLUDE_DIRS}
        )

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )

set (plugins_single_header
        ArduPilotPlugin
        ArduCopterIRLockPlugin
        GimbalSmall2dPlugin
        )

add_library(ArduCopterIRLockPlugin SHARED src/ArduCopterIRLockPlugin.cc)
target_link_libraries(ArduCopterIRLockPlugin ${GAZEBO_LIBRARIES})

add_library(ArduPilotPlugin SHARED src/ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES})

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
    target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
    install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()

install(TARGETS ArduCopterIRLockPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
install(TARGETS ArduPilotPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})

#install(DIRECTORY models DESTINATION ${GAZEBO_RESOURCE_PATH})
#install(DIRECTORY worlds DESTINATION ${GAZEBO_RESOURCE_PATH})