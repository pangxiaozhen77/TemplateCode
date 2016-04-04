/*
 * ObjectLocalisation.hpp
 *
 *  Created on: Apr. 4, 2016
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

namespace object_localisation {

/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class ObjectLocalisation
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ObjectLocalisation(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ObjectLocalisation();

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
