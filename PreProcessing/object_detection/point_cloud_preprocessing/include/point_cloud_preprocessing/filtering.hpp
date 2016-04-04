/*
 * filtering.hpp
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
#include <vector>

//PCL
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZRGB PointType;

namespace point_cloud_preprocessing {

/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class filtering
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  filtering();

  /*!
   * Destructor.
   */
  virtual ~filtering();

  /*!
   * Samples and Preprocesses PointClouds and saves them to preprocessed_cloud
   */
  bool getPreprocessedCloud(pcl::PointCloud<PointType>& preprocessed_cloud);

  /*!
   * Samples and Preprocesses PointClouds and saves them to preprocessed_cloud
   */
  bool setNumberOfAverageClouds(int number_of_averaged_clouds);

  /*!
   * Samples and Preprocesses PointClouds and saves them to preprocessed_cloud
   */
  bool setNumberOfMedianClouds(int number_of_median_clouds);

  /*!
   * Samples and Preprocesses PointClouds and saves them to preprocessed_cloud
   */
  bool setInputClouds(std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector);


 private:

  /*!
   * Applies median filter using number_of_clouds and writes resulting PointCloud in median_cloud
   * @return true if successful, false otherwise.
   */
  bool medianFilter(pcl::PointCloud<PointType>& median_cloud);

  /*!
   * Applies average filter on base_cloud using number_of_clouds and writes resulting PointCloud in base_cloud
   */
  bool averageFilter(pcl::PointCloud<PointType>& base_cloud);

 private:

  //! Number of clouds to be used for averaging filter
  int number_of_average_clouds_;

  //! Number of clouds to be used for median filter
  int number_of_median_clouds_;

  //! Number of clouds to be used for median filter
  int cloud_width_;

  //! Number of clouds to be used for median filter
  int cloud_height_;

  //! Number of clouds to be used for median filter
  int cloud_size_;

  std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector_;

};

} /* namespace */
