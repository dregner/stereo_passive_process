cmake_minimum_required(VERSION 3.20)
project(passive_stereo_capture CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

# ---- ROS 2 packages -------------------------------------------------------
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(nav_msgs       REQUIRED)
find_package(std_srvs       REQUIRED)
find_package(tf2            REQUIRED)
find_package(tf2_ros        REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- OpenCV ---------------------------------------------------------------
find_package(OpenCV REQUIRED)

# ---- CUDA -----------------------------------------------------------------
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      set(CMAKE_CUDA_ARCHITECTURES 87 72)
    else()
      set(CMAKE_CUDA_ARCHITECTURES 75 80 86 87)
    endif()
  endif()
  message(STATUS "CUDA enabled, archs: ${CMAKE_CUDA_ARCHITECTURES}")
endif()

# ---- Retinify (GPU stereo depth) ------------------------------------------
find_package(retinify REQUIRED)

# ---- Spinnaker SDK --------------------------------------------------------
if(NOT DEFINED SPINNAKER_ROOT)
  if(DEFINED ENV{SPINNAKER_ROOT})
    set(SPINNAKER_ROOT "$ENV{SPINNAKER_ROOT}")
  else()
    set(SPINNAKER_ROOT "/opt/spinnaker")
  endif()
endif()
if(NOT EXISTS "${SPINNAKER_ROOT}/include/Spinnaker.h")
  message(FATAL_ERROR "Spinnaker SDK not found at ${SPINNAKER_ROOT}")
endif()
add_library(Spinnaker::Spinnaker SHARED IMPORTED)
set_target_properties(Spinnaker::Spinnaker PROPERTIES
  IMPORTED_LOCATION "${SPINNAKER_ROOT}/lib/libSpinnaker.so"
  INTERFACE_INCLUDE_DIRECTORIES "${SPINNAKER_ROOT}/include"
)

# ---- ORB-SLAM3 ------------------------------------------------------------
if(NOT DEFINED ORB_SLAM3_ROOT)
  if(DEFINED ENV{ORB_SLAM3_ROOT})
    set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}")
  elseif(EXISTS "/home/jetson/ORB_SLAM3")
    set(ORB_SLAM3_ROOT "/home/jetson/ORB_SLAM3")
  else()
    set(ORB_SLAM3_ROOT "/home/daniel/ORB_SLAM3")
  endif()
endif()
if(NOT EXISTS "${ORB_SLAM3_ROOT}/include/System.h")
  message(FATAL_ERROR "ORB-SLAM3 not found at ${ORB_SLAM3_ROOT}")
endif()

# Find Eigen3, Sophus and Pangolin (required by ORB-SLAM3)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Sophus QUIET)
if(NOT Sophus_FOUND)
  # Fallback: use ORB-SLAM3 bundled Thirdparty
  set(SOPHUS_INCLUDE "${ORB_SLAM3_ROOT}/Thirdparty/Sophus")
  message(STATUS "Sophus not found via find_package; using ${SOPHUS_INCLUDE}")
else()
  set(SOPHUS_INCLUDE "")
endif()

add_library(ORB_SLAM3::ORB_SLAM3 SHARED IMPORTED)
set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ORB_SLAM3_ROOT}/include;${ORB_SLAM3_ROOT}/include/CameraModels;${SOPHUS_INCLUDE}"
)

# ---- libgpiod (for hardware trigger) -------------------------------------
find_package(PkgConfig QUIET)
pkg_check_modules(GPIOD QUIET libgpiod)
if(NOT GPIOD_FOUND)
  find_library(GPIOD_LIB NAMES gpiod HINTS /usr/lib /usr/local/lib)
  if(GPIOD_LIB)
    set(GPIOD_LIBRARIES ${GPIOD_LIB})
    set(GPIOD_FOUND TRUE)
  endif()
endif()

if(GPIOD_FOUND)
  message(STATUS "libgpiod found: ${GPIOD_LIBRARIES}")
  add_compile_definitions(HAVE_GPIOD=1)
else()
  message(WARNING "libgpiod not found on host system (will be active on Jetson if installed). Building without direct GPIO hardware trigger support.")
endif()

# ---- Sources --------------------------------------------------------------
set(SOURCES
  src/stereo_calib.cpp
  src/gpio_trigger.cpp
  src/spinnaker_grabber.cpp
  src/slam_worker.cpp
  src/disparity_worker.cpp
  src/preview_worker.cpp
  src/passive_stereo_node.cpp
  src/main.cpp
)

add_executable(passive_stereo_node ${SOURCES})

target_include_directories(passive_stereo_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
    ${ORB_SLAM3_ROOT}/Thirdparty/g2o
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
    /usr/local/cuda/include
)

ament_target_dependencies(passive_stereo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(CUDAToolkit REQUIRED)

target_link_libraries(passive_stereo_node
  Spinnaker::Spinnaker
  ORB_SLAM3::ORB_SLAM3
  retinify
  CUDA::cudart
  ${Pangolin_LIBRARIES}
  ${GPIOD_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)


# ── RPATH configuration ──────────────────────────────────────────────────
# Ensure installed binary finds non-system libraries:
# 1. ORB-SLAM3 and thirdparty libs (DBoW2, g2o)
# 2. Spinnaker SDK
# 3. Pangolin (found either in system, /usr/local/lib, or ORB-SLAM3/Thirdparty/Pangolin/build)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(RPATH_DIRS
  "${ORB_SLAM3_ROOT}/lib"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  "${ORB_SLAM3_ROOT}/Thirdparty/Pangolin/build"
  "${SPINNAKER_ROOT}/lib"
  "/usr/local/lib"
)

if(DEFINED Pangolin_DIR)
  get_filename_component(_pango_dir "${Pangolin_DIR}" DIRECTORY)
  list(APPEND RPATH_DIRS "${Pangolin_DIR}" "${_pango_dir}" "${_pango_dir}/lib")
endif()

# Check each library in Pangolin_LIBRARIES
foreach(_p_lib IN LISTS Pangolin_LIBRARIES)
  if(TARGET "${_p_lib}")
    get_target_property(_loc "${_p_lib}" IMPORTED_LOCATION_RELEASE)
    if(NOT _loc)
      get_target_property(_loc "${_p_lib}" IMPORTED_LOCATION)
    endif()
    if(_loc)
      get_filename_component(_d "${_loc}" DIRECTORY)
      list(APPEND RPATH_DIRS "${_d}")
    endif()
  elseif(EXISTS "${_p_lib}")
    get_filename_component(_d "${_p_lib}" DIRECTORY)
    list(APPEND RPATH_DIRS "${_d}")
  endif()
endforeach()

list(REMOVE_DUPLICATES RPATH_DIRS)
message(STATUS "passive_stereo_capture RPATH_DIRS: ${RPATH_DIRS}")

set_target_properties(passive_stereo_node PROPERTIES
  INSTALL_RPATH "${RPATH_DIRS}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# ---- Install --------------------------------------------------------------
install(TARGETS passive_stereo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Threads REQUIRED)
  add_executable(pipeline_queue_test tests/queue_test.cpp)
  target_include_directories(pipeline_queue_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(pipeline_queue_test Threads::Threads)
  add_test(NAME pipeline_queue_test COMMAND pipeline_queue_test)
endif()

ament_package()
