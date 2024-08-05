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

# Harmonic (default)
if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
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

# Make external include directories system includes to suppress warnings.
include_directories(SYSTEM ${RapidJSON_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${GST_INCLUDE_DIRS})

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

# Retrieve the list of current targets
get_property(current_targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
# Filter out targets that end with _uninstall
list(FILTER current_targets EXCLUDE REGEX "uninstall$")

# Apply compile options to each target, cannot use global compile options as gazebo don't comply with them
foreach(target ${current_targets})
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(${target} PRIVATE -Wshadow -Wformat)
    target_compile_options(${target} PRIVATE -Wcast-align -Woverloaded-virtual)
    target_compile_options(${target} PRIVATE -Wnull-dereference)
    target_compile_options(${target} PRIVATE -Wconversion)
    target_compile_options(${target} PRIVATE -Wmissing-declarations -Wmissing-include-dirs -Wredundant-decls)
    target_compile_options(${target} PRIVATE -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Wsign-promo -Wswitch -Wundef)
    target_compile_options(${target} PRIVATE -Wold-style-cast)
    target_compile_options(${target} PRIVATE -Werror)
    target_compile_options(${target} PRIVATE -Werror=return-type)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(${target} PRIVATE -Wuseless-cast )
      target_compile_options(${target} PRIVATE -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wstrict-null-sentinel)
    endif()

    target_compile_options(${target} PRIVATE -ffunction-sections -fdata-sections)
    target_link_options(${target} PRIVATE -Wl,--gc-sections)
  endif()
endforeach()

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