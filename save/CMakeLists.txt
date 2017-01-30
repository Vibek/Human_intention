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

find_package(OpenCV REQUIRED)

# Find PCL
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(/usr/include/openni /usr/include/nite /usr/include/ni)
rosbuild_add_executable(skeleton_tracker src/skeleton_tracker.cpp src/KinectController.cpp src/KinectDisplay.cpp src/BrazierCurve.cpp src/attention_map.cpp src/utils.cpp src/find_objects_pcl.cpp)
target_link_libraries(skeleton_tracker glut OpenNI )
target_link_libraries(skeleton_tracker ${OpenCV_LIBS})

rosbuild_add_executable(openni src/openni_tracker1.cpp)
target_link_libraries(openni glut OpenNI )
target_link_libraries(openni ${OpenCV_LIBS})

rosbuild_add_executable(hand src/Drinking_water.cpp)
target_link_libraries(hand glut OpenNI )
target_link_libraries(hand ${OpenCV_LIBS})

rosbuild_add_executable(kinect_aux_node src/kinect_aux_node.cpp)
target_link_libraries(kinect_aux_node -lusb-1.0)

rosbuild_add_executable(object src/find_objects_color.cpp)
target_link_libraries(object ${OpenCV_LIBS})

rosbuild_add_executable(svm src/classification_with_svm.cpp)
target_link_libraries(svm ${OpenCV_LIBS})

rosbuild_add_executable(tran src/training_with_svm.cpp)
target_link_libraries(tran ${OpenCV_LIBS})




