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
# Find Ignition dependencies

# Default version to Ignition Fortress
set(IGN_CMAKE_VER 2)
set(IGN_GAZEBO_VER 6)

if("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  # Edifice
  set(IGN_CMAKE_VER 2)
  set(IGN_GAZEBO_VER 5)
  message(STATUS "Compiling against Ignition Edifice")
elseif("$ENV{IGNITION_VERSION}" STREQUAL "garden")
  # Garden
  set(IGN_CMAKE_VER 2)
  set(IGN_GAZEBO_VER 7)
  message(STATUS "Compiling against Ignition Garden")
else() 
  # Default to Fortess
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(ignition-cmake${IGN_CMAKE_VER} REQUIRED)
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

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
