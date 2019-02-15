/*   
* ---  Pointcloud Pose Estimation of AGV Base  ---
* Created by: Tan You Liang (Feb 2019, Hope Technik Project)
* 
* 2D laser scan processing to pointcloud, identify robot base via line detection,
* filter out irrelavent data, and have checking mechanism to ensure accuracy of the line fitting. 
* conduct pose estimation of robot's base, with infomation of x, y, z r, p, y
*/



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// sensor msg
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>




class ObjectPoseEstimation
{
  private:

    ros::Publisher _pose_estimation_pub; 
    ros::Subscriber _urg_scan_sub;
    
    
    geometry_msgs::Vector3Stamped imu_rpy;
    // sensor_msgs::Imu imu_msg;             // to get imu_msg, and change its orientation
    // int is_encoderOdom_init;              // 1 if first odom value

    pcl::PointCloud<pcl::PointXYZ> cloudPrevFrame;    // Prev frame pointcloud
    tf::Transform tf_odom;
    float base_link_angVel;        // Base_link yaw angle turning angular velocity
    int cloud_count;               // track number of cloud being summed

    ros::Time odomTimeStmp;       // timestamp from encoder odom


  // ** Param Server
  protected:

    float turningRegTresh;        // max allowed angular vel tresh for turning robot to register pointcloud
    int timeStampCompensation;    // compensation of velodyne timestamp diff to the actual time stamp
    float testVal;                // valuable for testing

    //* Parse ROS launch parameters
    bool parseParams(const ros::NodeHandle& nh) {

      bool success = true;
      float fParam;
      int iParam;

      turningRegTresh = 9.0;
      timeStampCompensation = 0;

      if (nh.getParam("turningRegTresh", fParam)) {
        if (fParam < 0.01) {
          ROS_ERROR("Invalid min angular turning vel for registration Tresh parameter: %f (expected >= 0.01)", fParam);
          success = false;
        } else {
          turningRegTresh = fParam;
          ROS_INFO(" => Set turningRegTresh: %g", fParam);
        }
      }

      if (nh.getParam("timeStampCompensation", iParam)) {
        if (iParam > 1000000000) {
          ROS_ERROR("Invalid timeStampCompensation, which is recommended to be within 1x10^9 nsec: %i", iParam);
          success = false;
        } else {
          timeStampCompensation = iParam;
          ROS_INFO(" => Set timeStampCompensation: %i", iParam);
        }
      }


      if (nh.getParam("testVal", fParam)) {
        testVal = fParam;
        ROS_INFO(" => Set test val: %g", fParam);
      }

      ROS_INFO(" Done with param init");

      return success;
    }



  public:

    //setup pub and sub
    bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
    {
      
      if (!parseParams(privateNode)) {
        return false;
      }

      // --- Publish static orientation pointcloud with transformed input pcd based of IMUrpy.orientation
      _pub_cloudSummation =  node.advertise<sensor_msgs::PointCloud2> ("/cloud_summation", 1);

      // --- Source cloud from velodyne 
      _sub_pointCloud = node.subscribe<sensor_msgs::PointCloud2::Ptr>("/velodyne_points", 2, &PointCloudRegistration::cloudSummation_callback, this);
      
      // --- IMU rpy and rpy standard msg from IMU sensor
      // _sub_IMUrpy = node.subscribe<geometry_msgs::Vector3Stamped>("/imu/rpy", 10, &OdomHandler::imu_rpyCallback, this);
      // _sub_IMUmsg = node.subscribe<sensor_msgs::Imu>("/imu/imu", 10, &OdomHandler::imu_msgCallback, this);
      
      // --- Encoder odom msg from robot
      _sub_encoderOdom = node.subscribe<nav_msgs::Odometry::Ptr>("/odom", 10, &PointCloudRegistration::encoderOdom_Callback, this);

      cloud_count = 1;

      ROS_INFO("Done with Setup of Sub and Pub for node");
      return true;
    }



    // // sub to /imu/imu to get retrieve all info of sensor
    // void imu_msgCallback(const sensor_msgs::Imu::ConstPtr& imuIn){
    //   imu_msg = *imuIn;
    // }



    // // sub to /imu/ geometry_msgs/Vector3Stamped
    // void imu_rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imuIn){

    //   // ----------------------- IMU filtered --------------

    //   // // Managing TF publisher
    //   static tf::TransformBroadcaster br;
    //   tf::Transform transform;
    //   transform.setOrigin( tf::Vector3(0, 0, 0.0) );
    //   imu_rpy = *imuIn;
      
    //   // set value tf publisher
    //   tf::Quaternion q;
    //   q.setRPY(imu_rpy.vector.x, -imu_rpy.vector.y, -imu_rpy.vector.z); //rpy to quaternion
    //   transform.setRotation(q);
    //   // br.sendTransform(tf::StampedTransform(transform, imu_rpy.header.stamp, "odom", "base_link")); //use imu orientation as odom

    //   // // Managing IMU Publisher
    //   sensor_msgs::Imu imuOut;  // set value to new imu publisher
      
    //   imuOut.header = imu_rpy.header;

    //   imuOut.orientation.x = q.x();
    //   imuOut.orientation.y = q.y();
    //   imuOut.orientation.z = q.z();
    //   imuOut.orientation.w = q.w();

    //   imuOut.angular_velocity = imu_msg.angular_velocity;
    //   imuOut.linear_acceleration = imu_msg.linear_acceleration;
      
    //   ROS_INFO("publishing imu_filtered out");

    //   _pub_IMUfiltered.publish(imuOut);


    // }



    // transform pcd to imu rpy frame
    void cloudSummation_callback(const sensor_msgs::PointCloud2::Ptr _cloud){

      pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::fromROSMsg( *_cloud, *velo_cloud);

      Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 


      ROS_INFO("Robot Turning Speed: %lf", base_link_angVel);  
      ROS_INFO("odomTime: %i cloudTime: %i", odomTimeStmp.nsec, _cloud->header.stamp.nsec);  
      
      float yaw_timeCompensation = transformPredictionOnTimeDiff(odomTimeStmp.nsec, _cloud->header.stamp.nsec);




      // ==================  odom Transformation =====================

      double x, y, z, roll, pitch, yaw;
      tf::Matrix3x3( tf_odom.getRotation() ).getRPY(roll, pitch, yaw);
      tf::Vector3 vec = tf_odom.getOrigin();

      // Transformation
      transform.translation() << 0,0,0;//vec.x(), vec.y(), vec.z(); //TYL TEMP

      // The same rotation matrix as before; theta radians around Z axis
      transform.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
      transform.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
      transform.rotate (Eigen::AngleAxisf (yaw + yaw_timeCompensation, Eigen::Vector3f::UnitZ()));

      // // Executing the transformation
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*velo_cloud, *transformed_cloud, transform);
      cloudPrevFrame =  *transformed_cloud;


      // ==================== end odom Transformation =====================



      // ==================   ICP Transformation  =====================

      // match current transformed cloud with prev cloud
      Eigen::VectorXf icp_tf(6);
      icp_tf = icpMatching(transformed_cloud);

      transform = Eigen::Affine3f::Identity(); 

      // translation
      transform.translation() << icp_tf[0], icp_tf[1], icp_tf[2];

      // The same rotation matrix as before; theta radians around Z axis
      transform.rotate (Eigen::AngleAxisf (icp_tf[3], Eigen::Vector3f::UnitX()));
      transform.rotate (Eigen::AngleAxisf (icp_tf[4], Eigen::Vector3f::UnitY()));
      transform.rotate (Eigen::AngleAxisf (icp_tf[5], Eigen::Vector3f::UnitZ()));

      // // Executing the transformation
      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*transformed_cloud, *icp_transformed_cloud, transform);
      cloudPrevFrame =  *icp_transformed_cloud;


      // ==================== end ICP Transformation 2 =====================


      if ( fabs(base_link_angVel) <= turningRegTresh){
        // only output =) TBC

        // Output msg
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*transformed_cloud, msg);
        // pcl::toROSMsg(*icp_transformed_cloud, msg);
        msg.header.stamp = _cloud->header.stamp;
        msg.header.frame_id = _cloud->header.frame_id;
        _pub_cloudSummation.publish(msg);
      }

    }
  

    // Temporately ICP matching code 
    // compare current and prev pointcloud, and return transformation between both point cloud
    Eigen::VectorXf icpMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud){
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloudPrevFrame));

      icp.setInputSource(current_cloud);
      icp.setInputTarget(prev_cloud);
      pcl::PointCloud<pcl::PointXYZ> Final;
      icp.align(Final);

      std::cout << "-- ICP has converged:" << icp.hasConverged() << "   score: " <<  icp.getFitnessScore() << std::endl;

      Eigen::Matrix4f tf_matrix = icp.getFinalTransformation(); // 4x4 matrix
      Eigen::Matrix3f rot_matrix = tf_matrix.block<3,3>(0,0);   // 3x3 Rot Matrix
      Eigen::Vector3f rpy = rot_matrix.eulerAngles(0, 1, 2);    // 1x3 rpy rotation
      std::cout << "-- ICP 4x4 Matrix: \n" << tf_matrix << std::endl;

      Eigen::Vector3f xyz; 
      xyz << tf_matrix(0,3), tf_matrix(1,3), tf_matrix(2,3);    // 1x3 xyz trans
      Eigen::VectorXf vec_xyzrpy(6);

      vec_xyzrpy << xyz, rpy;  
      std::cout << "XYZRPY : \n" << vec_xyzrpy << std::endl;
    
      return vec_xyzrpy;
    }


    // Publish encoder odom msg to TF (camera_init to encoderOdom)
    void encoderOdom_Callback(const nav_msgs::Odometry::Ptr _encoderOdom){

      // // Managing TF publisher
      static tf::TransformBroadcaster br;

      float x, y, z, w;
      x = _encoderOdom->pose.pose.position.x;
      y = _encoderOdom->pose.pose.position.y;
      z = _encoderOdom->pose.pose.position.z;
      tf::Vector3 vec(x,y,z);

      x = _encoderOdom->pose.pose.orientation.x;
      y = _encoderOdom->pose.pose.orientation.y;
      z = _encoderOdom->pose.pose.orientation.z;
      w = _encoderOdom->pose.pose.orientation.w;
      tf::Quaternion quat(x,y,z,w);

      base_link_angVel = _encoderOdom->twist.twist.angular.z; // yaw angular vel
      odomTimeStmp = _encoderOdom->header.stamp;

      tf_odom.setOrigin(vec);
      tf_odom.setRotation(quat);

      ROS_INFO("odom callback: with time: %i speed: %f", odomTimeStmp.nsec, base_link_angVel);  


      br.sendTransform(tf::StampedTransform(tf_odom, _encoderOdom->header.stamp, "odom", "base_link")); //use imu orientation as odom

    }

    // ** PROTECTED
    // Compensate rotation diff based of diff in time stamp
    float transformPredictionOnTimeDiff(int t1, int t2){
      
      int time_diff = t2 - t1 + timeStampCompensation;

      if (abs(time_diff) > 600000000){ // to solve the digit prob. e.g: sec: 0 nsec:999 -> sec:1 nsec:1
        time_diff = 1000000000 + time_diff;
      }

      float rotate_compensation = base_link_angVel*float(time_diff)/1000000000;

      ROS_INFO("- Time Diff: %i ; yaw compensation: %f", time_diff, rotate_compensation);  

      return rotate_compensation;
    }


};

// // ==================================== End Class ===========================================





// // ======================================= Main ================================================

int main(int argc, char** argv){

  ros::init(argc, argv, "registration_node");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  // Get Arguments
  int path = 0; // default
  if (argc > 1) {
    path = std::atoi(argv[1]);
    std::cout << "Pcd path is: " << path << std::endl;
  }

  ROS_INFO("Using msg geometry_msgs/Vector3Stamped!!");

  PointCloudRegistration registration;
  if (registration.setup(node, privateNode)) {
    // successful initialization 
    ros::spin();
  }   

  ROS_ERROR("Error, clean exit!");
  return 0;
};