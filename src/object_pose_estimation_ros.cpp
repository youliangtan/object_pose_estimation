
/* 
 *  Created By: Tan You Liang, Feb 2019
 *  - for testing on ransac interested object identification and pose estimation
 *  - Created for Testing
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include <iostream>
#include <assert.h>
#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <object_pose_estimation.h>


// geometry_msgs/Pose2D.msg


// temp global var
pcl::PointCloud<pcl::PointXYZ>::Ptr hokoyu_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>); 
Eigen::Vector3f target_pose;


class PoseEstimationNode {
    public:
        PoseEstimationNode();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        
        ObjectPoseEstimate2D agv_laser_scan;

};


// init handler
PoseEstimationNode::PoseEstimationNode(): agv_laser_scan("src/object_pose_estimation/config/config.yaml") {
    std::cout<< "init class... " << std::endl;
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &PoseEstimationNode::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

}


// laserscan callback
void PoseEstimationNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    std::cout<< "Callback " << std::endl;


    sensor_msgs::PointCloud2 raw_cloud;

    // convert '/scan' to '/cloud'
    projector_.transformLaserScanToPointCloud("laser", *scan, raw_cloud, tfListener_);
    pcl::fromROSMsg(raw_cloud, *hokoyu_cloud);
    
    agv_laser_scan.reInit();
    agv_laser_scan.setInputCloud(hokoyu_cloud);
    agv_laser_scan.getTargetPose(&target_pose);
    agv_laser_scan.getTargetPointCloud(target);
    
    // Output msg
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*target, msg);
    msg.header.frame_id = "laser";
    point_cloud_publisher_.publish(msg);
}


int main(int argc, char** argv)
{
    std::cout<< "starting pose estimation node... " << std::endl;
    ros::init(argc, argv, "hokoyu_pose_estimation");
    PoseEstimationNode pose_estimation_node; 
    ros::spin();
    return 0;
}