cmake_minimum_required(VERSION 2.8.3)
project(object_pose_estimation)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG} -std=c++11")

add_definitions(-std=c++11 -Wall -Werror)

remove_definitions(-Werror)
message((none) "message to display" ...)


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
find_package(PCL 1.7 REQUIRED)
find_package(YAML-CPP REQUIRED yaml-cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs laser_geometry tf
  DEPENDS EIGEN3 PCL #OCTOMAP
)


link_directories(${PCL_LIBRARY_DIRS} ${YAML_CPP_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${YAML_CPP_DEFINITIONS} ${EIGEN3_DEFINITIONS} )
add_definitions( -march=native )

add_library(object_pose_estimation  src/object_pose_estimation.cpp )
target_link_libraries(object_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${YAML_CPP_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(object_pose_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(object_pose_estimation_ros src/object_pose_estimation_ros.cpp )
target_link_libraries(object_pose_estimation_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${YAML_CPP_LIBRARIES} object_pose_estimation )

add_executable(scan2pcd src/scan2pcd.cpp)
target_link_libraries(scan2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# set (CMAKE_CXX_STANDARD 11)


# project(object_pose_estimation)
# SET(BUILD_SHARED_LIBS ON)


# find_package(PCL 1.2 REQUIRED)
# find_package(YAML-CPP REQUIRED yaml-cpp)

# include_directories(${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS} ${YAML_CPP_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS} ${YAML_CPP_DEFINITIONS})

# add_executable (object_pose_estimation object_pose_estimation.cpp)
# target_link_libraries (object_pose_estimation ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})