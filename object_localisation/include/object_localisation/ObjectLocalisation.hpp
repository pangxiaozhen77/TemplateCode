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
#include <pcl/PolygonMesh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <geometry_msgs/PoseArray.h>

namespace object_localisation {
/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class ObjectLocalisation
{
  typedef pcl::PointXYZRGB PointType;

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

  bool preprocess();

  bool computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);

  bool computeNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud);

  bool computeKeypoints(const pcl::PointCloud<PointType>::ConstPtr &cloud);

  bool computeMesh(const pcl::PointCloud<PointType>::ConstPtr &cloud);

  bool computeROPSDescriptor();

  bool computeFPFHDescriptor();

  bool computeFPFHLRFs();

  bool FPFHcorrespondence();

  bool loadModelData(int model_number);

  bool Clustering();

  bool ICP(int instance);

  bool Output();

  bool Visualisation();

 private:


  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Grid map publisher.
  ros::Publisher publisher_;

  pcl::PointCloud<PointType>::Ptr preprocessed_cloud_ptr_;

  pcl::PointCloud<PointType>::Ptr preprocessed_model_ptr_;

  pcl::PointCloud<PointType>::Ptr transposed_model_ptr_;

  pcl::PointCloud<PointType>::Ptr keypoint_cloud_ptr_;

  pcl::PointCloud<PointType>::Ptr keypoint_model_ptr_;

  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  std::vector <pcl::PointCloud <PointType> > cloud_vector_;

  pcl::PolygonMesh::Ptr triangles_;

  pcl::PointCloud<pcl::Histogram <135> >::Ptr RoPS_histograms_;

  pcl::PointCloud<pcl::ReferenceFrame>::Ptr RoPS_LRFs_;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_signature_scene_;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_signature_model_;

  pcl::CorrespondencesPtr model_scene_correspondence_;

  pcl::PointCloud<pcl::ReferenceFrame>::Ptr FPFH_LRF_scene_;

  pcl::PointCloud<pcl::ReferenceFrame>::Ptr FPFH_LRF_model_;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations_;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > refined_rototranslations_;

  std::vector<pcl::Correspondences> clustered_correspondences_;

  unsigned int preprocessed_size_;

  unsigned int keypoints_size_;

  double cloud_resolution_;

  double sqr_correspondence_distance_;

  int max_;

  int model_index_;

  geometry_msgs::PoseArray model_poses_;

  std::string model_path_;
  std::string save_path_;

//Number of Models to match
  int number_of_models_;
//Filtering Parameters
  int number_of_average_clouds_;
  int number_of_median_clouds_;
  float z_threshold_;
  float planarSegmentationTolerance_;
  int min_number_of_inliers_;
  float xmin_;
  float xmax_;
  float ymin_;
  float ymax_;
  float zmin_;
  float zmax_;

//Meshing Parameters
  int number_of_neighbors_normal_;
  float max_edge_length_;
  float mu_;
  int max_nearest_neighbors_mesh_;
  float max_surface_angle_;
  float  min_angle_;
  float  max_angle_;
  bool  normal_consistency_;

//Keypoint Detection Parameters
  double normal_radius_;
  double salient_radius_;
  double border_radius_;
  double non_max_radius_;
  double gamma_21_;
  double gamma_32_;
  double min_neighbors_;
  int threads_;

// Parameters for RoPS-Descriptor.
  float RoPS_radius_;
  bool crops_;

// Parameters for FPFH-Descriptor
  double FPFH_radius_;

// Local Reference Frames
  double lrf_search_radius_;

// Hough clustering
  double bin_size_;
  double threshold_;

// Refinement and Validation
  float max_fitness_score_;

// Visualisation
  double inlier_dist_;
  double offset_;
};

} /* namespace */
