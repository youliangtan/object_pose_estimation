/* 
 *  Created By: Tan You Liang, Sept 2018
 *  - Node to capture laserscan topic, convert to pointcloud2, then save to .pcd format
 *  - Created for Testing
*/

// Convert laser to pointcloud

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


// temp global var
pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
int output_count;


class LaserConversion {
    public:
        LaserConversion();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};


// init handler
LaserConversion::LaserConversion(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/front_scan", 100, &LaserConversion::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}


// laserscan callback
void LaserConversion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;

    // convert '/scan' to '/cloud'
    projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
    pcl::fromROSMsg(cloud, *output_cloud);

    point_cloud_publisher_.publish(cloud);
}


// SIGINT to save pcd
void save_interrupt(int s){
    // output file to current working dir
    std::string idx = std::to_string(output_count); //convert int to str
    pcl::io::savePCDFileASCII ("SavedCloud" + idx + ".pcd", *output_cloud);
    std::cout<<" - SUCCESS!!! output .pcd file is saved!! "<<std::endl;
    output_count++;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_conversion");
    LaserConversion laser2pointcloud;
    signal(SIGINT,save_interrupt);
    ros::spin();
    return 0;
}






// ============================================ to be junk ========================================








// #include <octomap/octomap.h>

// using namespace std;


// // save latest 3d point cloud to local
// void octo_callback(const sensor_msgs::PointCloud2ConstPtr& _cloud){
//     pcl::fromROSMsg( *_cloud, *output_cloud);
//     cout<<"point cloud loaded, point size = "<< output_cloud->points.size()<<endl;
// }

// // interrupt handler


// int main( int argc, char** argv )
// {
//     char c;
//     // start ros
//     ros::init(argc, argv, "pcd2octo");
//     ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2, octo_callback);
//     ros::Rate r(100);
    
//     // create > save .bt file interrupt
//     signal(SIGINT,save_interrupt);

//     ros::spin();
    
//     return 0;

// }


