cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

# Create utility library
add_library(lego_loam_utility src/utility.cpp)
target_link_libraries(lego_loam_utility ${catkin_LIBRARIES} yaml-cpp)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection lego_loam_utility ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation lego_loam_utility ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization lego_loam_utility ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam yaml-cpp)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion lego_loam_utility ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
