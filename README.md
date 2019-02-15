# object_pose_estimation
Simple 2D pointcloud target object pose estimator
(Testing code!)

*Developing!!!!!!!!!!!!!*

## Getting Started

Use ROS Hokoyu Driver Package [here](https://github.com/ros-drivers/urg_node).
`roslaunch urg_node urg_lidar.launch ip_address:=XXXXXXX`

Conversion from laserscan to pointcloud
` rosrun object_pose_estimation scan2pcd.cpp `

Main Pose Estimation Bin
` rosrun object_pose_estimation obj_pose_estimation.cpp `


## What's Going on
Via hokoyu scan input, convert to pointcloud, use ransac line detector to find the target flat object, then find the boundary of the line, at last estimate the pose of the detected object.

## Some Back Up Code
PCL Ransac line fitting
```
cd pcl_ransac/build
cmake ..
make -j4
./random_sample_consensus -input <input.pcd>
```
