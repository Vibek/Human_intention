cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#include($ENV{ROS_ROOT}/catkin/cmake/debug_message.cmake)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#find_package(libusb REQUIRED)
#include_directories(${libusb_1.0_INCLUDE_DIRS} /usr/include/libusb-1.0)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
include_directories(/usr/include/openni /usr/include/nite /usr/include/ni)
rosbuild_add_executable(skeleton_tracker src/skeleton_tracker.cpp src/KinectController.cpp src/KinectDisplay.cpp )
target_link_libraries(skeleton_tracker glut OpenNI )
rosbuild_add_executable(kinect_aux_node src/kinect_aux_node.cpp)
#rosbuild_add_executable(kinect_aux_node src/libusb.h)
target_link_libraries(kinect_aux_node -lusb-1.0)
#rosbuild_add_executable(jointposition src/jointposition.cpp)
#rosbuild_add_executable(Drinking_water src/Drinking_water.cpp)
#rosbuild_add_executable(openni_tracker1 src/openni_tracker1.cpp)
#target_link_libraries(openni_tracker1 ${OPENNI_LIBRARIES})

#find_package(PCL REQUIRED)
#link_directories(${LIBUSB_1_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})
