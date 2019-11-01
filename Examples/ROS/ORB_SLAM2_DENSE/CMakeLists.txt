cmake_minimum_required(VERSION 2.8)
project(orb_slam2_dense)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(OpenCV_DIR /usr/local/include)
# for Youcompleteme surpport
set( CMAKE_EXPORT_COMPILE_COMMANDS "ON" )
#include_directories(/usr/include/pcl-1.7)
#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()

#MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    tf
    pcl_ros
    pcl_conversions
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
set(OpenCV_DIR "/usr/share/OpenCV")
   find_package(OpenCV 2.4 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
   message(STATUS "OpenCV Find Version: ${OpenCV_VERSION}")
   message(STATUS "  OpenCV Include: ${OpenCV_INCLUDE_DIRS}")
   message(STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}")
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
set(PCL_DIR "/usr/share/pcl-1.7")
find_package( PCL 1.7.2 EXACT REQUIRED )

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

catkin_package(
    INCLUDE_DIRS
      ${PROJECT_SOURCE_DIR}/../../../include
    #  LIBRARIES nav_goal
    CATKIN_DEPENDS
       roscpp
       std_msgs
       sensor_msgs
       geometry_msgs
       cv_bridge
       tf
    #  DEPENDS system_lib
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/ORB_SLAM2_DENSE
    ${PROJECT_SOURCE_DIR}/../../../
    ${PROJECT_SOURCE_DIR}/../../../include
    ${Pangolin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(LIBS 
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
#    ${PROJECT_SOURCE_DIR}/../../../lib/libboost_filesystem.so
#    ${PROJECT_SOURCE_DIR}/../../../lib/libboost_system.so
#    ${PROJECT_SOURCE_DIR}/../../../lib/libopencv_core3.so
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Node for monocular camera
#rosbuild_add_executable(Mono
#src/ros_mono.cc
#)
add_executable(Mono
    src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
#rosbuild_add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)
add_executable(MonoAR
    src/AR/ros_mono_ar.cc
    src/AR/ViewerAR.h
    src/AR/ViewerAR.cc
)

target_link_libraries(MonoAR
${LIBS}
)

# Node for stereo camera
#rosbuild_add_executable(Stereo
#src/ros_stereo.cc
#)
add_executable(Stereo
    src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
#rosbuild_add_executable(RGBD
#src/ros_rgbd.cc
#)
add_executable(RGBD
    src/ros_rgbd.cc
    src/ORB_SLAM2_DENSE/utils/message_utils.cpp
)

target_link_libraries(RGBD
${LIBS}
)

