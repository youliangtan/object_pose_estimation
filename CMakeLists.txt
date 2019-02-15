cmake_minimum_required(VERSION 2.8.3)
project(object_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  tf2
  laser_geometry
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs laser_geometry tf
  DEPENDS EIGEN3 PCL #OCTOMAP
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions( -march=native )

# add_executable(obj_pose_estimation src/obj_pose_estimation.cpp)
# target_link_libraries(obj_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scan2pcd src/scan2pcd.cpp)
target_link_libraries(scan2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})