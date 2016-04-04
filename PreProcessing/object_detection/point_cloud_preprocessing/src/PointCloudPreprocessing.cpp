/*
 * PointCloudPreprocessing.cpp
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

#include "point_cloud_preprocessing/filtering.hpp"
#include "point_cloud_preprocessing/PointCloudPreprocessing.hpp"

namespace point_cloud_preprocessing {

std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector;

void saveCloud(const sensor_msgs::PointCloud2& cloud){
  pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
  pcl::fromROSMsg(cloud, new_cloud);
  cloud_vector.push_back(new_cloud);
}

PointCloudPreprocessing::PointCloudPreprocessing(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  //Filtering Parameters
  int number_of_average_clouds = 15;
  int number_of_median_clouds = 5;
  ros::Rate r(60);

  // initializing publisher and subscriber
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, saveCloud);
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("object_detection/preprocessedCloud", 1, true);

  //Save clouds from topic to cloud_vector
  while (cloud_vector.size() < number_of_average_clouds + number_of_median_clouds)
  {
   ros::spinOnce();
   r.sleep();
  }

  // initialize point clouds
  pcl::PointCloud<pcl::PointXYZRGB> preprocessed_cloud;

  filtering filtering;
  filtering.setNumberOfMedianClouds(number_of_median_clouds);
  filtering.setNumberOfAverageClouds(number_of_average_clouds);
  filtering.setInputClouds(cloud_vector);
  std::cout << "3" << std::endl;
  filtering.getPreprocessedCloud(preprocessed_cloud);
  std::cout << "4" << std::endl;
  publish(preprocessed_cloud);

  // saving clouds to PCD
  pcl::io::savePCDFileASCII ("PointCloud_preprocessed_unorganized.pcd", preprocessed_cloud);

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
