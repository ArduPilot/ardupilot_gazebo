cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ardupilot_sitl_gazebo)

#------------------------------------------------------------------------
# Compile as C++14

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 2.8.0 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find ignition-gazebo
ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

#--------------------------------------
# Find RapidJSON
find_package(RapidJSON REQUIRED)

#======================================
# Build plugin

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/Socket.cpp
)

target_include_directories(ArduPilotPlugin PUBLIC
  include
	${IGNITION-GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(ArduPilotPlugin PUBLIC
  ${IGNITION-GAZEBO_LIBRARIES}
)
