# object_pose_estimation
Simple 2D pointcloud target object pose estimator
(Testing code!)

## Getting Started

Use ROS Hokoyu Driver Package [here](https://github.com/ros-drivers/urg_node).
`roslaunch urg_node urg_lidar.launch ip_address:=XXXXXXX`


## What's Goinf on
Via hokoyu scan input, convert to pointcloud, use ransac line detector to find the target flat object, then find the boundary of the line, at last estimate the pose of the detected object.
