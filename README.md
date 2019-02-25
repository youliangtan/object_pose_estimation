# object_pose_estimation

ROS 2D pointcloud target object pose estimator. Line-based target identifier with user input parameters.

Via hokoyu scan input, convert /scan to /pointcloud. Subsequent pcl lib functions were used to indentify the target. Ransac line detector is used to find the target flat object, which helps in identify the boundary of the line, eventually estimate the pose `[x, y, theta]` of the detected object. 

For usecase of realtime ROS processing, multi-samples are taken to compute moving averaging to smoothern the output pose estimation result. To obtain an optimal result, parameters can be further tuned in `config/config.yaml`.

![alt text](/resources/rviz_example.png?)

*Developing!!!!!!!!!!!!!*

## Getting Started

```
cd catkin_ws
catkin_make --pkg object_pose_estimation
source devel/setup.bash
```

## Run the code

**Use ROS Hokoyu Driver Package [here](https://github.com/ros-drivers/urg_node).**
```
roslaunch urg_node urg_lidar.launch ip_address:=XXXXXXX
```

**Main Pose Estimation ROS2 Node**
``` 
rosrun object_pose_estimation object_pose_estimation_ros
```

**Visualize on Rviz**
visualize topic on `/scan`, `/target_pose`, `/target_cloud`.
```
rviz -f laser
```


## Notes
 - `ObjectPoseEstimate2D` class is constructed with mainly depends on `pcl` library.
 - User can test the single `.pcd` sample with the code on `object_pose_estimation.cpp`, run via `rosrun object_pose_estimation object_pose_estimation -input SavedCloud0.pcd`
 - Conversion from laserscan to pointcloud: `rosrun object_pose_estimation scan2pcd.cpp`.