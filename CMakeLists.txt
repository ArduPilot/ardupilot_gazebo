cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ardupilot_gazebo)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

# --------------------------------------------------------------------------- #
# Compile as C++14.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# --------------------------------------------------------------------------- #
# Find RapidJSON.
find_package(RapidJSON REQUIRED)

# --------------------------------------------------------------------------- #
# Build plugin.

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/Socket.cpp
    src/Util.cc
)
target_include_directories(ArduPilotPlugin PRIVATE
  include
)
target_link_libraries(ArduPilotPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(ParachutePlugin
  SHARED
  src/ParachutePlugin.cc
)
target_include_directories(ParachutePlugin PRIVATE
  include
)
target_link_libraries(ParachutePlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# --------------------------------------------------------------------------- #
# Install.

install(
  TARGETS
  ArduPilotPlugin
  ParachutePlugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# --------------------------------------------------------------------------- #
# Register as an ament package if ament_cmake is available.
if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

  ament_package()
endif()