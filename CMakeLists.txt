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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.
set(PACKAGES
  sdformat13
  gz-cmake3
  gz-sim7
  gz-sensors7
  gz-math7
  gz-utils2
  gz-common5
  gz-msgs9
  gz-rendering7
  gz-transport12
  gz-plugin2
  # Add more packages as needed
)

foreach(PACKAGE ${PACKAGES})
  find_package(${PACKAGE} REQUIRED)
  string(REGEX REPLACE "[0-9]+" "" PACKAGE_UPPER ${PACKAGE})
  string(REPLACE "-" "_" PACKAGE_UPPER ${PACKAGE_UPPER})
  string(TOUPPER ${PACKAGE_UPPER} PACKAGE_UPPER)
  set(${PACKAGE_UPPER}_VER ${${PACKAGE}_VERSION_MAJOR})
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(gtk3 REQUIRED IMPORTED_TARGET gtk+-3.0)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)


find_package(OpenCV REQUIRED)


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

add_library(GstCameraPlugin 
  SHARED 
  src/GstCameraPlugin.cpp
)
target_include_directories(GstCameraPlugin PRIVATE
  include
)
target_link_libraries(GstCameraPlugin PRIVATE
  ${OpenCV_LIBS}
  PkgConfig::gtk3
  PkgConfig::gstreamer
  PkgConfig::gstreamer-sdp
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-video
  sdformat13::sdformat13
  gz-sim7::gz-sim7
  gz-sensors7::gz-sensors7
  gz-math7::gz-math7
  gz-utils2::gz-utils2
  gz-common5::gz-common5
  gz-msgs9::gz-msgs9
  gz-rendering7::gz-rendering7
  gz-transport12::gz-transport12
  gz-plugin2::gz-plugin2
)

# --------------------------------------------------------------------------- #
# Install.

install(
  TARGETS
  ArduPilotPlugin
  ParachutePlugin
  GstCameraPlugin
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