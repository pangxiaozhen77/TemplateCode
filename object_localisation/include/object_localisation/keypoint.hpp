/*
 * keypoint.hpp
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
#include <vector>

//PCL
#include <pcl/common/common_headers.h>


typedef pcl::PointXYZRGB PointType;

namespace point_cloud_keypoint {

/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class keypoint
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  keypoint();

  /*!
   * Destructor.
   */
  virtual ~keypoint();

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool compute(pcl::PointCloud<pcl::PointXYZRGB>& keypoint_cloud);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setModelResolution(double model_resolution);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setNormalRadius(double normal_radius);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setBorderRadius(double border_radius);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setSalientRadius(double salient_radius);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setNonMaxRadius(double non_max_radius);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setGamma21(double gamma_21);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setGamma32(double gamma_32);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMinNeighbors(double min_neighbors);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setThreads(double threads);

 private:

  //! Number of clouds to be used for averaging filter
  double model_resolution_;
  double normal_radius_;
  double border_radius_;
  double salient_radius_;
  double non_max_radius_;
  double gamma_21_;
  double gamma_32_;
  double min_neighbors_;
  int threads_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;

};

} /* namespace */
