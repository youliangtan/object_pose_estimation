
/*  CPP code on ROS Node execution of 'ObjectPoseEstimate2D' class
 *  
 *  Created By: Tan You Liang, Feb 2019
 *  - For testing on 2D pose estimation of targeted object (line)
 *  - Created for Testing
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <assert.h>
#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <object_pose_estimation.hpp>


#define SKIP_PUB_FRAME 20 // changeable int to reduce pub rate for '/ur10/target_pose' topic

std::string *yaml_path;

class PoseEstimationNode {
    public:
        PoseEstimationNode();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    protected:
        pcl::PointCloud<pcl::PointXYZ>::Ptr hokoyu_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target;
        Eigen::Vector3f target_pose;
    
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher pose_publisher_;
        ros::Publisher point_cloud_publisher_;
        ros::Publisher pose_publisher_2d;
        ros::Subscriber scan_sub_;
        int pub_frame_num; // keep track of pub frame num
        
        ObjectPoseEstimate2D agv_laser_scan;
};


// ************************************************************************************************************************************************************** //
// ************************************************************************************************************************************************************** //



// init handler
PoseEstimationNode::PoseEstimationNode(): agv_laser_scan(*yaml_path) {
    std::cout<< "Init ROS Pose Estimation Node" << std::endl;

    hokoyu_cloud = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
    target = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;

    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &PoseEstimationNode::scanCallback, this);
    pose_publisher_ = node_.advertise<geometry_msgs::PoseStamped> ("/target_pose", 100, false);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 100, false);
    // for arm manipulator control
    pose_publisher_2d = node_.advertise<geometry_msgs::Pose2D> ("/ur10/target_pose", 100, false);

    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    pub_frame_num = 0;
}


// laserscan callback
void PoseEstimationNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    std::cout<< "\n[ ---------------------------- Scan Callback --------------------------]" << std::endl;
    sensor_msgs::PointCloud2 raw_cloud;

    // convert '/scan' to '/cloud'
    projector_.transformLaserScanToPointCloud("laser", *scan, raw_cloud, tfListener_);
    pcl::fromROSMsg(raw_cloud, *hokoyu_cloud);
    
    agv_laser_scan.reInit();
    agv_laser_scan.setInputCloud(hokoyu_cloud);
    agv_laser_scan.applyMovingAvgFiltering();
    agv_laser_scan.getTargetPose(&target_pose);
    agv_laser_scan.getTargetPointCloud(target);

    // Output PointCloud msg
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*target, pc_msg);
    pc_msg.header.frame_id = "laser";
    point_cloud_publisher_.publish(pc_msg);

    // Output Pose Stamped Msg
    tf::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(0, 0, target_pose[2]); //rpy to quaternion
    
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0.0) );
    transform.setRotation(quat_tf);
    quaternionTFToMsg(quat_tf , quat_msg);
    
    geometry_msgs::PoseStamped poseStamp_msg;
    poseStamp_msg.pose.position.x = target_pose[0];
    poseStamp_msg.pose.position.y = target_pose[1];
    poseStamp_msg.pose.position.z = 0;
    poseStamp_msg.pose.orientation = quat_msg;
    poseStamp_msg.header.frame_id = "laser";
    pose_publisher_.publish(poseStamp_msg);

    // Output 2D Pose Msg
    if (pub_frame_num > SKIP_PUB_FRAME){
        geometry_msgs::Pose2D pose_2d_msg;
        pose_2d_msg.x = target_pose[0];
        pose_2d_msg.y = target_pose[1];
        pose_2d_msg.theta = target_pose[2];
        pose_publisher_2d.publish(pose_2d_msg);
        pub_frame_num = 0;
    }
    else{
        pub_frame_num++;
    }
}


// ***************************************************************************************************
/////////////////////////////////////////// Main Function  ///////////////////////////////////////////
// ***************************************************************************************************


int main(int argc, char** argv)
{
    std::cout<< "Number of Arg: " << argc << std::endl;

    if ( argc < 2){
        ROS_ERROR(  "Number of Arg is %d ", argc );
        ROS_ERROR(  "Invalid num of arg, use:"
                    "\n > rosrun object_pose_estimation object_pose_estimation_ros $YAML_PATH"
                    "\n route: src/object_pose_estimation/config/config.yaml");
        exit(0);
    }

    yaml_path= new std::string(argv[1]);
    std::cout << " Yaml path is " << *yaml_path << std::endl;
    ROS_WARN(  "YAML Path is %s ", argv[1] );

    std::cout<< "starting pose estimation node... " << std::endl;
    ros::init(argc, argv, "hokoyu_pose_estimation");
    PoseEstimationNode pose_estimation_node; 
    ros::spin();
    return 0;
}