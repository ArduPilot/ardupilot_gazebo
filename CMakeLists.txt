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

# ign_find_package(sdformat11 REQUIRED VERSION 11.2.2)
# set(SDF_VER ${sdformat11_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
# ign_find_package(ignition-common4 REQUIRED COMPONENTS all)
# set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gazebo
ign_find_package(ignition-gazebo5 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
# ign_find_package(ignition-math6 REQUIRED VERSION 6.8)
# set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
# ign_find_package(ignition-msgs7 REQUIRED VERSION 7.1)
# set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-physics
# ign_find_package(ignition-physics4 REQUIRED)
# set(IGN_PHYSICS_VER ${ignition-physics4_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
# ign_find_package(ignition-plugin1 REQUIRED)
# set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-sensors
# ign_find_package(ignition-sensors5 REQUIRED)
# set(IGN_SENSORS_VER ${ignition-sensors5_VERSION_MAJOR})

#--------------------------------------
# Find RapidJSON
find_package(RapidJSON REQUIRED)

#======================================
# Find Qt
# find_package(Qt5 REQUIRED COMPONENTS
#     Core
#     Gui
#     Qml
#     Quick
#     QuickControls2
#     Widgets
# )

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/Socket.cpp
)

target_include_directories(ArduPilotPlugin PUBLIC
  # ${PROJECT_SOURCE_DIR}
  include
# 	${SDFORMAT-INCLUDE_DIRS}
#   ${IGNITION-COMMON_INCLUDE_DIRS}
	${IGNITION-GAZEBO_INCLUDE_DIRS}
# 	${IGNITION-MATH_INCLUDE_DIRS}
# 	${IGNITION-MSGS_INCLUDE_DIRS}
# 	${IGNITION-PHYSICS_INCLUDE_DIRS}
# 	${IGNITION-SENSORS_INCLUDE_DIRS}
# 	${IGNITION-TRANSPORT_INCLUDE_DIRS}
#   ${RAPIDJSON_INCLUDE_DIRS}
)

target_link_libraries(ArduPilotPlugin PUBLIC
  # ${SDFORMAT-LIBRARIES}
  # ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
  # ${IGNITION-MATH_LIBRARIES}
  # ${IGNITION-MSGS_LIBRARIES}
  # ${IGNITION-PHYSICS_LIBRARIES}
  # ${IGNITION-SENSORS_LIBRARIES}
  # ${IGNITION-TRANSPORT_LIBRARIES}
  # Qt5::Core
  # Qt5::Gui
  # Qt5::Qml
  # Qt5::Quick
)
