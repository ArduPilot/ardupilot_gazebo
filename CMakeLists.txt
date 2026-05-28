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

# ROS Lyrical installs Gazebo through vendor packages under /opt/ros/lyrical.
# Make the underlying Gazebo package configs visible for plain `cmake ..` on
# Lyrical hosts even when the ROS setup file has not been sourced. This is
# intentionally after the optional ament lookup so unsourced standalone builds
# do not require ROS Python paths.
set(GZ_LYRICAL_PREFIX "/opt/ros/lyrical")
if(("$ENV{GZ_VERSION}" STREQUAL "" OR "$ENV{GZ_VERSION}" STREQUAL "lyrical")
    AND EXISTS "${GZ_LYRICAL_PREFIX}")
  file(GLOB GZ_LYRICAL_VENDOR_PREFIXES
    LIST_DIRECTORIES true
    "${GZ_LYRICAL_PREFIX}/opt/*_vendor"
  )
  list(PREPEND CMAKE_PREFIX_PATH ${GZ_LYRICAL_VENDOR_PREFIXES})
endif()

set(GZ_VERSION "$ENV{GZ_VERSION}")
if("${GZ_VERSION}" STREQUAL "")
  if(EXISTS
      "${GZ_LYRICAL_PREFIX}/opt/gz_sim_vendor/lib/cmake/gz-sim/gz-sim-config.cmake")
    set(GZ_VERSION "lyrical")
  else()
    set(GZ_VERSION "harmonic")
  endif()
endif()

if("${GZ_VERSION}" STREQUAL "lyrical")
  # ROS Lyrical / Gazebo Jetty vendor packages.
  if(${ament_cmake_FOUND})
    find_package(gz_cmake_vendor REQUIRED)
    find_package(gz_common_vendor REQUIRED)
    find_package(gz_rendering_vendor REQUIRED)
    find_package(gz_sim_vendor REQUIRED)
  endif()

  find_package(gz-cmake REQUIRED)
  set(GZ_CMAKE_VER "")

  gz_find_package(gz-common REQUIRED)
  set(GZ_COMMON_VER "")

  gz_find_package(gz-rendering REQUIRED)
  set(GZ_RENDERING_VER "")

  gz_find_package(gz-sim REQUIRED)
  set(GZ_SIM_VER "")

  message(STATUS "Compiling against ROS Lyrical Gazebo vendor packages")
elseif("${GZ_VERSION}" STREQUAL "harmonic")
  # Harmonic (default)
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
elseif("${GZ_VERSION}" STREQUAL "ionic")
  # Ionic
  find_package(gz-cmake4 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake4_VERSION_MAJOR})

  gz_find_package(gz-common6 REQUIRED)
  set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

  gz_find_package(gz-rendering9 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering9_VERSION_MAJOR})
  
  gz_find_package(gz-sim9 REQUIRED)
  set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Ionic")
elseif("${GZ_VERSION}" STREQUAL "jetty")
  # Jetty
  find_package(gz-cmake REQUIRED)
  set(GZ_CMAKE_VER "")

  gz_find_package(gz-common REQUIRED)
  set(GZ_COMMON_VER "")

  gz_find_package(gz-rendering REQUIRED)
  set(GZ_RENDERING_VER "")
  
  gz_find_package(gz-sim REQUIRED)
  set(GZ_SIM_VER "")

  message(STATUS "Compiling against Gazebo Jetty")
elseif("${GZ_VERSION}" STREQUAL "garden")
  # Garden
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering7 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
  
  gz_find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: ${GZ_VERSION}")
endif()

# --------------------------------------------------------------------------- #
find_package(RapidJSON REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)


# --------------------------------------------------------------------------- #
# Build plugin.

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/SocketUDP.cc
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

add_library(CameraZoomPlugin
  SHARED
  src/CameraZoomPlugin.cc
)
target_include_directories(CameraZoomPlugin PRIVATE
  include
)
target_link_libraries(CameraZoomPlugin PRIVATE
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(GstCameraPlugin
  SHARED
  src/GstCameraPlugin.cc
)
target_include_directories(GstCameraPlugin PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
target_link_libraries(GstCameraPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${OpenCV_LIBS}
  ${GST_LINK_LIBRARIES}
)

# --------------------------------------------------------------------------- #
# Install.

install(
  TARGETS
  ArduPilotPlugin
  ParachutePlugin
  CameraZoomPlugin
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
