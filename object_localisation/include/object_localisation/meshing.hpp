/*
 * meshing.hpp
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
#include <pcl/surface/gp3.h>

typedef pcl::PointXYZRGB PointType;

namespace point_cloud_meshing {

/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class meshing
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  meshing();

  /*!
   * Destructor.
   */
  virtual ~meshing();

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool compute(pcl::PolygonMesh& triangles);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setNumberOfNeighborsNormal(int number_of_neighbors_normal);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMaxEdgeLength(float max_edge_length);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMu(float mu);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMaxNearestNeighborsMesh(int max_nearest_neighbors_mesh);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMaxSurfaceAngle(float max_surface_angle);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMinAngle(float min_angle);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setMaxAngle(float max_angle);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setNormalConsistency(bool normal_consistency);

  /*!
   * Applies median-, average-, passthrough- and segmentation-filter on the input clouds
   */
  bool setInputCloud(pcl::PointCloud<PointType>::Ptr cloud);


 private:

  //! Number of clouds to be used for averaging filter
  int number_of_neighbors_normal_;

  //! Number of clouds to be used for averaging filter
  float max_edge_length_;

  //! Number of clouds to be used for averaging filter
  float mu_;

  //! Number of clouds to be used for averaging filter
  int max_nearest_neighbors_mesh_;

  //! Number of clouds to be used for averaging filter
  float max_surface_angle_;

  //! Number of clouds to be used for averaging filter
  float min_angle_;

  //! Number of clouds to be used for averaging filter
  float max_angle_;

  //! Number of clouds to be used for averaging filter
  bool normal_consistency_;

  //! Number of clouds to be used for averaging filter
  pcl::PointCloud<PointType>::Ptr cloud_;

};

} /* namespace */
