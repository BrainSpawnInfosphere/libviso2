cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

project (libviso2)

# include directory
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# OpenCV
find_package(OpenCV 2.3.1 REQUIRED)
if(OpenCV_FOUND)
	message("======== ${OpenCV_VERSION} Found ========= ")
endif(OpenCV_FOUND)


# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/lib/*.cpp")

# build library
rosbuild_add_library(viso2 ${LIBVISO2_SRC_FILES})

# stereo demo program
rosbuild_add_executable(stereo_demo "src/stereo_demo.cpp")
target_link_libraries(stereo_demo  ${OpenCV_LIBS} viso2)

# mono demo program
rosbuild_add_executable(mono_demo "src/mono_demo.cpp")
target_link_libraries(mono_demo  ${OpenCV_LIBS} viso2)

# mono node
rosbuild_add_executable(mono_node "src/mono_node.cpp")
target_link_libraries(mono_node  ${OpenCV_LIBS} viso2)

