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
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 3.0.0 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

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
	${GZ-SIM_INCLUDE_DIRS}
)

target_link_libraries(ArduPilotPlugin PUBLIC
  ${GZ-SIM_LIBRARIES}
)
