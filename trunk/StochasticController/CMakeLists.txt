cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(/home/shridhar/Dropbox/Personal/fuerte_workspace/stocRHC/alglib; /home/shridhar/Dropbox/Personal/fuerte_workspace/stocRHC/; /home/shridhar/Dropbox/Personal/fuerte_workspace/stocRHC/include/stocRHC/; /usr/local/include/player-3.0/)
link_directories(/home/shridhar/Dropbox/Personal/fuerte_workspace/stocRHC/; /usr/local/lib/; /usr/lib/)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(demo src/demo.cpp src/Build.cpp src/Environment.cpp src/flat_control.cpp src/gamma.cpp src/Integrator.cpp src/MPN2D.cpp src/Obstacle.cpp src/saturate.cpp src/Unicycle.cpp src/vicon_multi.cpp)
#target_link_libraries(demo alglib ViconDataStreamSDK_CPP config++ playerc++ boost_thread boost_signals playerc playerinterface playerwkb playercommon)
#rosbuild_add_executable(OctoroachSOC src/OctoroachController.cpp src/ComputeVW.cpp)
#rosbuild_add_executable(ViconPublisher src/ViconPublisher.cpp)
#target_link_libraries(ViconPublisher ViconDataStreamSDK_CPP)
#rosbuild_add_boost_directories()
rosbuild_add_executable(SOCdemo src/socdemo.cpp src/stoc_controller.cpp)
