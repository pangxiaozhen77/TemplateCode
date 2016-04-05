/*
 * meshing.cpp
 *
 *  Created on: Apr. 5, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "object_localisation/keypoint.hpp"
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>

namespace point_cloud_keypoint{

keypoint::keypoint()
        : model_resolution_(0.0017674),
          normal_radius_(4),
          border_radius_(1),
          salient_radius_(6),
          non_max_radius_(4),
          gamma_21_ (0.975),
          gamma_32_ (0.975),
          min_neighbors_ (5),
          threads_ (4),
          input_cloud_()
{
}

keypoint::~keypoint()
{
}

bool keypoint::compute(pcl::PointCloud<pcl::PointXYZRGB>& keypoint_cloud){

      // set scale parameters with model resolution
      salient_radius_ = salient_radius_ * model_resolution_;
      non_max_radius_ = non_max_radius_ * model_resolution_;
      normal_radius_ = normal_radius_ * model_resolution_;
      border_radius_ = border_radius_ * model_resolution_;

      // Compute keypoints
      pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

      iss_detector.setSearchMethod (tree);
      iss_detector.setSalientRadius (salient_radius_);
      iss_detector.setNonMaxRadius (non_max_radius_);
      iss_detector.setNormalRadius (normal_radius_);
      iss_detector.setBorderRadius (border_radius_);
      iss_detector.setThreshold21 (gamma_21_);
      iss_detector.setThreshold32 (gamma_32_);
      iss_detector.setMinNeighbors (min_neighbors_);
      iss_detector.setNumberOfThreads (threads_);
      iss_detector.setInputCloud (input_cloud_);
      iss_detector.compute (keypoint_cloud);

  return true;
}

  bool keypoint::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){
    input_cloud_ = input_cloud;
    return true;
  }

  bool keypoint::setModelResolution(double model_resolution){
    model_resolution_ = model_resolution;
    return true;
  }

  bool keypoint::setNormalRadius(double normal_radius){
    normal_radius_ = normal_radius;
    return true;
  }

  bool keypoint::setBorderRadius(double border_radius){
    border_radius_ = border_radius;
    return true;
  }

  bool keypoint::setSalientRadius(double salient_radius){
    salient_radius_ = salient_radius;
    return true;
  }

  bool keypoint::setNonMaxRadius(double non_max_radius){
    non_max_radius_ = non_max_radius;
    return true;
  }

  bool keypoint::setGamma21(double gamma_21){
    gamma_21_ = gamma_21;
    return true;
  }

  bool keypoint::setGamma32(double gamma_32){
    gamma_32_ = gamma_32;
    return true;
  }

  bool keypoint::setMinNeighbors(double min_neighbors){
    min_neighbors_ = min_neighbors;
    return true;
  }

  bool keypoint::setThreads(double threads){
    threads_ = threads;
    return true;
  }

} /* namespace */
