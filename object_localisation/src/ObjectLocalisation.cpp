/*
 * ObjectLocalisation.cpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include "object_localisation/filtering.hpp"
#include "object_localisation/ObjectLocalisation.hpp"

std::stack<clock_t> tictoc_stack;

void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "Time elapsed for filtering: "
            << ((double)(clock()-tictoc_stack.top()))/CLOCKS_PER_SEC
            << std::endl;
  tictoc_stack.pop();
}

namespace point_cloud_preprocessing {

std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector;

void saveCloud(const sensor_msgs::PointCloud2& cloud){
  pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
  pcl::fromROSMsg(cloud, new_cloud);
  cloud_vector.push_back(new_cloud);
}

ObjectLocalisation::ObjectLocalisation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  //Filtering Parameters
  int number_of_average_clouds = 15;
  int number_of_median_clouds = 5;
  float z_threshold = 0.02;
  float planarSegmentationTolerance = 0.05;
  float xmin = -1;
  float xmax = 1;
  float ymin = -1;
  float ymax = 1;
  float zmin = 0.5;
  float zmax = 3;
  ros::Rate r(60);

  //Building boundary vector
  std::vector<float> boundaries;
  boundaries.push_back(xmin);
  boundaries.push_back(xmax);
  boundaries.push_back(ymin);
  boundaries.push_back(ymax);
  boundaries.push_back(zmin);
  boundaries.push_back(zmax);

  // Timer on
  tic();

  // initializing publisher and subscriber
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, saveCloud);
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("object_detection/preprocessedCloud", 1, true);

  //Save clouds from topic to cloud_vector
  while (cloud_vector.size() < number_of_average_clouds + number_of_median_clouds)
  {
   ros::spinOnce();
   r.sleep();
  }

  std::cout << "Clouds are sampled."
            << "  width = " << cloud_vector[0].width
            << "  height = " << cloud_vector[0].height
            << "  size = " << cloud_vector[0].size() << std::endl;

  // initialize point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB> ());

  filtering filtering;
  filtering.setNumberOfMedianClouds(number_of_median_clouds);
  filtering.setNumberOfAverageClouds(number_of_average_clouds);
  filtering.setInputClouds(cloud_vector);
  filtering.setClippingBoundaries(boundaries);
  filtering.setZThreshold(z_threshold);
  filtering.setPlanarSegmentationTolerance(planarSegmentationTolerance);
  filtering.getPreprocessedCloud(preprocessed_cloud_ptr);
  publish(*preprocessed_cloud_ptr);

  //Timer off
  toc();

  // saving clouds to PCD
  pcl::io::savePCDFileASCII ("PointCloud_preprocessed_unorganized.pcd", *preprocessed_cloud_ptr);

  // shut down node
  ros::requestShutdown();
}

PointCloudPreprocessing::~PointCloudPreprocessing()
{
}

void PointCloudPreprocessing::publish(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  pcl::PCLPointCloud2 cloud_pcl;
  pcl::toPCLPointCloud2(cloud, cloud_pcl);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(cloud_pcl, cloud_msg);

  // publishing cloud
  publisher_.publish(cloud_msg);
}

} /* namespace */
