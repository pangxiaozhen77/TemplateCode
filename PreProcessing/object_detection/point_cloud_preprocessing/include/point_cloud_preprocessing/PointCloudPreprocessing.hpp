/*
 * PointCloudPreprocessing.hpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>

// STD
#include <string>

//PCL
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>


typedef pcl::PointXYZRGB PointType;

namespace point_cloud_preprocessing {

/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class PointCloudPreprocessing
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloudPreprocessing(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PointCloudPreprocessing();

  /*!
   * publish message to topic
   */
  void publish(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

 private:

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Grid map publisher.
  ros::Publisher publisher_;

};

} /* namespace */
