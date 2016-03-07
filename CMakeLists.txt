cmake_minimum_required(VERSION 2.8.7)
project(loam_velodyne)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)


rosbuild_init() 

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(Eigen3 REQUIRED)

# don't use opencv as found by any prior running of find_package
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)

set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
set(CMAKE_PREFIX_PATH "$ENV{HOME}/special/install")
find_package(OpenCV 3.0 REQUIRED)

# restore CMAKE_PREFIX_PATH and other cached variables
# so nothing other package finds this opencv
set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})

unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)

find_package(Ceres 1.2.3 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Messages & Services
rosbuild_genmsg()


add_definitions(${PCL_DEFINITIONS})


# SSE flags
rosbuild_check_for_sse()
#add_definitions("-DUSE_ROS")
#add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x -Wall -g -O3"
) 

# Set source files
set(loam_SOURCE_FILES
${PROJECT_SOURCE_DIR}/src/loam/scanRegistration.cpp
${PROJECT_SOURCE_DIR}/src/loam/laserOdometry.cpp
${PROJECT_SOURCE_DIR}/src/loam/laserMapping.cpp
${PROJECT_SOURCE_DIR}/src/loam/transformMaintenance.cpp
${PROJECT_SOURCE_DIR}/src/loam/loam_wrapper.cpp
${PROJECT_SOURCE_DIR}/src/loam/icp.cpp
)

set(util_SOURCE_FILES
${PROJECT_SOURCE_DIR}/src/KITTI_util/KITTI.cpp
${PROJECT_SOURCE_DIR}/src/KITTI_util/toMATLAB.cpp
${PROJECT_SOURCE_DIR}/src/util/matrix.cpp
)
set(SOURCE_FILES
  ${loam_SOURCE_FILES}
  ${util_SOURCE_FILES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

# build shared library.
rosbuild_add_library(loam SHARED ${SOURCE_FILES})
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})



# build to file
#rosbuild_add_executable(main_loam src/main_loam.cpp)
#target_link_libraries(main_loam loam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#rosbuild_add_executable(main_KITTI ${catkin_LIBRARIES} src/main_KITTI.cpp)
#target_link_libraries(main_KITTI ${catkin_LIBRARIES} ${OpenCV_LIBS} loam ${PCL_LIBRARIES} ${CERES_LIBRARIES})

rosbuild_add_executable(main_testICP ${catkin_LIBRARIES} src/main_testICP.cpp)
target_link_libraries(main_testICP ${catkin_LIBRARIES} ${OpenCV_LIBS} loam ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#rosbuild_add_executable(main_ceres ${catkin_LIBRARIES} src/main_ceres.cpp)
#target_link_libraries(main_ceres ${catkin_LIBRARIES} ${OpenCV_LIBS} loam ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#rosbuild_add_executable(main_eval src/util/evaluate_odometry.cpp)
#target_link_libraries(main_eval loam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
