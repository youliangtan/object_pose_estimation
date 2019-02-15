
/* 
 *  Created By: Tan You Liang, Feb 2019
 *  - for testing on ransac interested object identification and pose estimation
 *  - Created for Testing
*/


// TOBEREMOVE means for line endpoints visualization

#include <iostream>
#include <string>
#include <math.h>    


#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


// clustering and filtering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <boost/thread/thread.hpp>
#include <Eigen/Dense>


#define PI 3.14159265


// TODO: addd param file

// TO BE REMOVED!!
pcl::PointCloud<pcl::PointXYZ>::Ptr line_points(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointXYZ> *line_centers (new std::vector<pcl::PointXYZ>);
std::vector<float> *lines_length (new std::vector<float>);
int temp_line_idx = 0;




pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
                                                  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > *lines,
                                                  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > *clusters
                                                  ){


  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->addCoordinateSystem (1.0, "axis");
  viewer->setBackgroundColor (0.01, 0.01, 0.01);

  // Original cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, source_cloud_color_handler, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  


  // visualizting clusters
  int cluster_idx=0;
  for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator points = clusters->begin(); points != clusters->end(); ++points) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (*points, 0, 255, 0);
    std::string idx = std::to_string(cluster_idx); //convert int to str
    viewer->addPointCloud<pcl::PointXYZ> (*points, source_cloud_color_handler2, "cluster cloud" + idx);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cluster cloud" + idx);
    cluster_idx++;
  }
  
  // visualizting lines
  int line_idx=0;
  for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator points = lines->begin(); points != lines->end(); ++points) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (*points, 255, 0, 0);
    std::string idx = std::to_string(line_idx); //convert int to str
    viewer->addPointCloud<pcl::PointXYZ> (*points, source_cloud_color_handler3, "line cloud" + idx);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "line cloud" + idx);
    line_idx++;
  }

  // TODO: visualize texts
  for (int idx = 0; idx < lines_length->size (); ++idx){
    viewer->addText3D( "line" + std::to_string(idx) + " length m: " + std::to_string(lines_length->at(idx)), line_centers->at(idx), 0.08, 0.0, 1.0, 0.0, "line"+ std::to_string(idx));
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0,0,6,0,0,0);
  }

  // visualize line endpoints
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler4 (line_points, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (line_points, source_cloud_color_handler4, "endpoints");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "endpoints");


  return (viewer);
}



// identify all line's endpoints
// 2D plane only
void get_line_endpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf coeff){
  
  // init
  float score, score_max, score_min;
  score = cloud->points[0].x * coeff[3]  + cloud->points[0].y * coeff[4];
  score_max = score, 
  score_min = score;
  float x_min, y_min, x_max, y_max;
  x_min = cloud->points[0].x;
  y_min = cloud->points[0].y;
  x_max = cloud->points[0].x;
  y_max = cloud->points[0].y;

  for (int i = 1; i < cloud->points.size (); ++i){
    score = (cloud->points[i].x * coeff[3]) + (cloud->points[i].y * coeff[4]);
    // std::cout << "point :" << cloud->points[i].x * coeff[3] << " "  << cloud->points[i].y << std::endl;  
    // std::cout << "Score :" << score << " \t| x, Min, Max " << x_min << " " << x_max << std::endl;  

    if ( score > score_max ){
      score_max = score;
      x_max = cloud->points[i].x;
      y_max = cloud->points[i].y;
    }

    if ( score < score_min ){
      score_min = score;
      x_min = cloud->points[i].x;
      y_min = cloud->points[i].y;
    }
  }

  std::cout << "Score {min, max}: " << score_min << " " << score_max << std::endl;  
  std::cout << "Min {x, y} :" << x_min << " " << y_min << std::endl;
  std::cout << "Max {x, y} :" << x_max << " " << y_max << std::endl;

  float length; //todo remove sq rt
  length = sqrt( (x_max - x_min)*(x_max - x_min) + (y_max - y_min)*(y_max - y_min) );
  std::cout << " - length: " <<  length <<std::endl;

  // TOBEREMOVED
  // insert 2 endpoints for visualization
  line_points->points[temp_line_idx].x = x_min;
  line_points->points[temp_line_idx].y = y_min;
  line_points->points[temp_line_idx].z = 0;
  temp_line_idx++;
  line_points->points[temp_line_idx].x = x_max;
  line_points->points[temp_line_idx].y = y_max;
  line_points->points[temp_line_idx].z = 0;
  temp_line_idx++;


  pcl::PointXYZ line_center;
  line_center.x = (x_max + x_min )/2;
  line_center.y = (y_max + y_min )/2;
  line_center.z = 0;
  line_centers->push_back ( line_center );  
  lines_length->push_back ( length );
  
  // get value of theta 
  float theta;
  // TODOL take  care of coeff[3] when = 0
  theta = atan (coeff[4]/coeff[3]) * 180 / PI;
  printf (" - line theta is %f degrees\n", theta );

}


// TOBEREMOVED
// for visualization purpose
void init_get_line_endpoints(int size){

  // Fill in the cloud data
  line_points->width  = size;
  line_points->height = 1;
  line_points->points.resize (line_points->width * line_points->height);

}



// clustering and filtering
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
  
  // // outliner filtering
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (input_cloud);
  // sor.setMeanK (20);
  // sor.setStddevMulThresh (1);
  // sor.filter (*cloud_filtered);
  // std::cout << " Filtered cloud from " << input_cloud->size() << " to " << cloud_filtered->size() << std::endl;

  // // Create Cluster init
  // Creating the KdTree object for the search method of the extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.04); // 10cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (25000);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // clusterize each plane
  tree->setInputCloud (input_cloud); //TODO check if theres any use of kd tree func
  ec.setSearchMethod (tree);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  std::cout << "Num of Clusters: " << cluster_indices.size () << std::endl;
  // TOBEREMOVED
  init_get_line_endpoints( cluster_indices.size()*2 );

  // extract and visualize cluster segmentation for each plane
  int clusterNum = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    //create cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (input_cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "\n- PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    // update to list of clusters
    clusters.push_back ( cloud_cluster );  
    clusterNum ++;
  }
  return clusters;
}



// Line Fitting
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> line_fitting(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > *clusters){
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lines;

  for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator cloud = clusters->begin(); cloud != clusters->end(); ++cloud) {
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (*cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;
    Eigen::VectorXf coeff;  // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z] (unit vector)


    //ransac
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    ransac.setDistanceThreshold (.04); // so call error allowance for laser scan
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coeff);

    std::cout << "Line coeff: " << coeff[0] << " " << coeff[1] << " " << coeff[3] << " " << coeff[4] << std::endl;

    /// find lines' end points
    pcl::copyPointCloud<pcl::PointXYZ>(**cloud, inliers, *target);
    
    // // outliner filtering
    float dist_coeff;
    dist_coeff = (coeff[0]*coeff[0] + coeff[1]*coeff[1])* 2.7; // 0.1 is approx
    std::cout << "Distance coeff: " << dist_coeff << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (target);
    sor.setMeanK (20);
    sor.setStddevMulThresh (1.6*dist_coeff);
    sor.filter (*target);

    get_line_endpoints(target, coeff);

    lines.push_back ( target );  
    
  }
  return lines;
}
  



// //get pose estimation
// void get_pose_estimation(){

// }


// ------------------ Main Function -------------------
int main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_clusters;
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_lines;


  if (argc < 3 || pcl::console::find_switch (argc, argv, "-h") )
  {
      cout<<" - Run this script to process .pcd file to ransac seg "<<endl;
      cout<<"Usage: ./random_sample_consensus -l -input <input_file>"<<endl;
      return -1;
  }

  //get arg pcdinput 
  if (pcl::console::find_argument (argc, argv, "-input") >= 0){
      int input_idx = pcl::console::find_argument (argc, argv, "-input") + 1;
      std::string input_file = argv[input_idx];
      std::cout<<"File Path: "<<input_file<<std::endl;
      pcl::io::loadPCDFile<pcl::PointXYZ> ( input_file, *cloud );
      std::cout<<"Point cloud loaded, point size = "<<cloud->points.size()<<std::endl;
  } 
  else{
    std::cout<<"No Input PCD File, pls input via '-input' "<<std::endl;
    exit(0);
  }

  cloud_clusters = object_clustering(cloud);
  cloud_lines = line_fitting(&cloud_clusters);
  std::cout << "Clusters vector size: " << cloud_clusters.size () << std::endl;

  // visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = simpleVis(cloud, &cloud_lines, &cloud_clusters);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }