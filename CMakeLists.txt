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

if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
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
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic")
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
elseif("$ENV{GZ_VERSION}" STREQUAL "jetty")
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
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
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
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

# --------------------------------------------------------------------------- #
find_package(RapidJSON REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)


# --------------------------------------------------------------------------- #
# Find protobuf and abseil (required for macOS)
find_package(Protobuf CONFIG REQUIRED)
if(APPLE)
  find_package(absl REQUIRED)
endif()

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
  protobuf::libprotobuf
)

# On macOS, explicitly link abseil components needed by protobuf
# These are required when using protobuf 22+ which depends on abseil
if(APPLE)
  # Find Homebrew prefix (supports both Intel and Apple Silicon)
  execute_process(
    COMMAND brew --prefix
    OUTPUT_VARIABLE HOMEBREW_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  target_link_libraries(ArduPilotPlugin PRIVATE
    ${HOMEBREW_PREFIX}/lib/libabsl_log_internal_message.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_log_internal_check_op.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_log_internal_format.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_log_internal_globals.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_strings.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_str_format_internal.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_raw_logging_internal.dylib
    ${HOMEBREW_PREFIX}/lib/libabsl_log_severity.dylib
  )
endif()

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