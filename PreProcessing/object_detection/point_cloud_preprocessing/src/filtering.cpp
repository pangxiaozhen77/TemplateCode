/*
 * PointCloudPreprocessing.cpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "point_cloud_preprocessing/filtering.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace point_cloud_preprocessing {

filtering::filtering()
    :  number_of_average_clouds_(0),
       number_of_median_clouds_(1),
       cloud_width_(0),
       cloud_height_(0),
       cloud_size_(0),
       cloud_vector_()
{
}

filtering::~filtering()
{
}


bool filtering::getPreprocessedCloud(pcl::PointCloud<PointType>& preprocessed_cloud){

  std::cout << "4.1" << std::endl;
if(!medianFilter(preprocessed_cloud))
  return false;
std::cout << "4.2" << std::endl;
if(!averageFilter(preprocessed_cloud))
  return false;
std::cout << "4.3" << std::endl;

  return true;
}


bool filtering::setNumberOfAverageClouds(int number_of_average_clouds){
  number_of_average_clouds_ = number_of_average_clouds;
  return true;
}


bool filtering::setNumberOfMedianClouds(int number_of_median_clouds){
  if (number_of_median_clouds % 2 == 0){
    std::cout << "ERROR: must input an odd number of clouds" << std::endl;
    return false;
  }
  number_of_median_clouds_ = number_of_median_clouds;
  return true;
}

bool filtering::setInputClouds(std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector){
  cloud_vector_ = cloud_vector;
  return true;
}

bool filtering::medianFilter(pcl::PointCloud<PointType>& median_cloud){

  //get the cloud dimensions
  if (cloud_width_ == 0 || cloud_height_ == 0){
  cloud_width_ = cloud_vector_[0].width;
  cloud_height_ = cloud_vector_[0].height;
  cloud_size_ = cloud_width_ * cloud_height_;
  }

  double z_values [number_of_median_clouds_];
  double z_sorted [number_of_median_clouds_];

  for(int i_point = 0; i_point < cloud_size_; i_point++)
  {
    for (int j_cloud = 0; j_cloud < number_of_median_clouds_; j_cloud++)
    {
      z_values[j_cloud] = cloud_vector_[j_cloud].points[i_point].z;
      z_sorted[j_cloud] = z_values[j_cloud];
    }
    std::sort(z_sorted, z_sorted + number_of_median_clouds_);
    for (int k = 0; k < number_of_median_clouds_; k++)
    {
      if(z_sorted[(number_of_median_clouds_-1)/2] == z_values[k])
      {
        cloud_vector_[0].points[i_point] = cloud_vector_[k].points[i_point];
      }
    }
  }
  median_cloud = cloud_vector_[0];
  return true;
}

bool filtering::averageFilter(pcl::PointCloud<PointType>& base_cloud){
  //get the cloud dimensions
  if (cloud_width_ == 0 || cloud_height_ == 0){
  cloud_width_ = base_cloud.width;
  cloud_height_ = base_cloud.height;
  cloud_size_ = cloud_width_ * cloud_size_;
  }

  pcl::PointCloud<pcl::PointXYZRGB> unorganized_cloud;

  for (int k = 0; k < number_of_average_clouds_; ++k)
    {
        //iterate over points
        for (int i = 0; i < cloud_width_; i++)
        {
          for (int j = 0; j < cloud_height_; j++)
          {
            pcl::PointXYZRGB point_base_cloud = base_cloud(i,j);
            pcl::PointXYZRGB point_new_cloud = cloud_vector_[k](i,j);

            if (point_base_cloud.z != 0 && point_new_cloud.z != 0 && std::abs(point_base_cloud.z - point_new_cloud.z) < 0.02)
            {
              point_base_cloud.x = (point_base_cloud.x * (k-1) + point_new_cloud.x)/k;
              point_base_cloud.y = (point_base_cloud.y * (k-1) + point_new_cloud.y)/k;
              point_base_cloud.z = (point_base_cloud.z * (k-1) + point_new_cloud.z)/k;
              point_base_cloud.r = (point_base_cloud.r * (k-1) + point_new_cloud.r)/k;
              point_base_cloud.g = (point_base_cloud.g * (k-1) + point_new_cloud.g)/k;
              point_base_cloud.b = (point_base_cloud.b * (k-1) + point_new_cloud.b)/k;

            }
            else if (point_new_cloud.z != 0)
            {
              point_base_cloud.x = point_new_cloud.x;
              point_base_cloud.y = point_new_cloud.y;
              point_base_cloud.z = point_new_cloud.z;
              point_base_cloud.r = point_new_cloud.r;
              point_base_cloud.g = point_new_cloud.g;
              point_base_cloud.b = point_new_cloud.b;
            }

            base_cloud(i,j) = point_base_cloud;

            if (k == number_of_average_clouds_-1)
            {
              unorganized_cloud.push_back(point_base_cloud);
            }

          }
        }
    }
  // write unorganized cloud
  base_cloud.clear();
  base_cloud = unorganized_cloud;

  return true;
}

} /* namespace */
