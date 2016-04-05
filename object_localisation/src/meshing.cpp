/*
 * meshing.cpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "object_localisation/meshing.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <iostream>


namespace point_cloud_meshing{

meshing::meshing()
      : number_of_neighbors_normal_(30),
        max_edge_length_(0.025),
        mu_(1.5),
        max_nearest_neighbors_mesh_(50),
        max_surface_angle_(M_PI/4),
        min_angle_(5*M_PI/180),
        max_angle_(160*M_PI/180),
        normal_consistency_(false),
        cloud_()
{
}

meshing::~meshing()
{
}

bool meshing::compute(pcl::PolygonMesh& triangles){

  //Clone PointCloud to XYZ Cloud for normal estimation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_xyz->resize(cloud_->size());
  copyPointCloud(*cloud_, *cloud_xyz);

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_xyz);
  n.setInputCloud (cloud_xyz);
  n.setSearchMethod (tree);
  n.setKSearch (number_of_neighbors_normal_);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Apply greedy projection
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (max_edge_length_);

  // Set typical values for the parameters
  gp3.setMu (mu_);
  gp3.setMaximumNearestNeighbors (max_nearest_neighbors_mesh_);
  gp3.setMaximumSurfaceAngle(max_surface_angle_); // 45 degrees
  gp3.setMinimumAngle(min_angle_); // 10 degrees
  gp3.setMaximumAngle(max_angle_); // 120 degrees
  gp3.setNormalConsistency(normal_consistency_);
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  return true;
}


bool meshing::setNumberOfNeighborsNormal(int number_of_neighbors_normal){
  number_of_neighbors_normal_ = number_of_neighbors_normal;
  return true;
}

bool meshing::setMaxEdgeLength(float max_edge_length){
  max_edge_length_ = max_edge_length;
  return true;
}


bool meshing::setMu(float mu){
  mu_ = mu;
  return true;
}

bool meshing::setMaxNearestNeighborsMesh(int max_nearest_neighbors_mesh){
  max_nearest_neighbors_mesh_ =  max_nearest_neighbors_mesh;
  return true;
}


bool meshing::setMaxSurfaceAngle(float max_surface_angle){
  max_surface_angle_ = max_surface_angle;
  return true;
}

bool meshing::setMinAngle(float min_angle){
  min_angle_ = min_angle;
  return true;
}


bool meshing::setMaxAngle(float max_angle){
  max_angle_ = max_angle;
  return true;
}


bool meshing::setNormalConsistency(bool normal_consistency){
  normal_consistency_ = normal_consistency;
  return true;
}

bool meshing::setInputCloud(pcl::PointCloud<PointType>::Ptr cloud){
  cloud_ = cloud;
  return true;
}

} /* namespace object_loclisation*/
