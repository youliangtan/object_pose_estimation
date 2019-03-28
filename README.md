# object_pose_estimation

ROS 2D pointcloud target object pose estimator package. A line-based target identifier with user input parameters.

Via a 2d lidar scan input, 'PCL' lib functions were used to indentify the target. PCL Ransac line detector is used to find the target flat object, which helps in identify the boundary of the line, eventually output a estimated pose of the detected object: `[midpoint_x, midpoint_y, theta]`.

For realtime ROS processing use case, multi-samples are taken to compute moving averaging to smoothern the output pose estimation result. To obtain an optimal result, parameters can be further tuned in `config/config.yaml`.

![alt text](/resources/rviz_example.png?)


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

```
# Visualize topic on `/scan`, `/target_pose`, `/target_cloud`.
rviz -f laser
```

**How to use it**

By stating object (line) feature which you wish to detect in the `config.yaml` file, the ros node will be able to find the pose of the target object which matches the described features. After rosrun the code, user can try it out by placing a target in front of the lidar. A pink line will be appeared on RVIZ, which indicates the target line. 

```
# Try 
rosrun echo /ur10/target_pose  
```

User (in this case, a ur10) is able to get the current pose of the the target object with a format of `[x, y, theta]`. Else, this topic will return a msg of `[0,0,0]`, which shows no target found.


*If all these works, This package works!! & You are good to Go!!*



## Notes
 - `ObjectPoseEstimate2D` class is constructed with mainly depends on `pcl` library.
 - User can test the single `.pcd` sample with the code on `object_pose_estimation.cpp`, run via `rosrun object_pose_estimation object_pose_estimation -input SavedCloud0.pcd`
 - Conversion from laserscan to pointcloud: `rosrun object_pose_estimation scan2pcd.cpp`.
 - Change `SKIP_PUB_FRAME` in ros cpp file to change the pub rate
 - 'applyMovingAvgFiltering' in the lib handles high and low pass filter, which detects 'jump' here. To change the filtering param, edit the yaml file.

## TODO
- Add getROI()
- output twist info? stable score? to know the changes
- Update readme: add video
