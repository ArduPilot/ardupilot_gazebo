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
find_package(ignition-cmake3 3.0.0 REQUIRED)
set(GZ_CMAKE_VER ${ignition-cmake3_VERSION_MAJOR})

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find ignition-gazebo
gz_find_package(ignition-gazebo7 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo7_VERSION_MAJOR})

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
