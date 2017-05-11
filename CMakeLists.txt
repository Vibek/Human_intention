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

include_directories(${PCL_INCLUDE_DIRS})

find_package(Qt4 REQUIRED)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

find_package(OpenCV REQUIRED)

include_directories(/usr/include/openni /usr/include/nite /usr/include/ni /usr/include/qwtplot3d-qt4)
rosbuild_add_executable(skeleton_tracker src/skeleton_tracker.cpp src/KinectController.cpp src/KinectDisplay.cpp src/BrazierCurve.cpp src/attention_map.cpp src/utils.cpp)
# src/find_objects_pcl.cpp src/find_objects.cpp)
target_link_libraries(skeleton_tracker glut OpenNI )
target_link_libraries(skeleton_tracker ${OpenCV_LIBS})
target_link_libraries(skeleton_tracker ${QT_LIBRARIES})

rosbuild_add_executable(kinect_aux_node src/kinect_aux_node.cpp)
target_link_libraries(kinect_aux_node -lusb-1.0)

rosbuild_add_executable(object src/find_objects_color.cpp)
target_link_libraries(object ${OpenCV_LIBS})

# Find PCL
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#rosbuild_add_executable(Tracker_object2 src/Tracker_object2.cpp)
#target_link_libraries(Tracker_object2 ${PCL_LIBRARIES} ${OpenCV_LIBS})

#rosbuild_add_executable(test_image src/test_image.cpp)
#target_link_libraries(test_image ${PCL_LIBRARIES} ${OpenCV_LIBS})

