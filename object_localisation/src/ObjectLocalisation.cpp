/*
 * ObjectLocalisation.cpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/iss_3d.h>
#include <ros/package.h>
#include <string>


#include "object_localisation/filtering.hpp"
#include "object_localisation/meshing.hpp"
#include "object_localisation/ObjectLocalisation.hpp"
#include "object_localisation/rops_estimation.h"

std::stack<clock_t> tictoc_stack;

void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "Time elapsed for object localisation: "
            << ((double)(clock()-tictoc_stack.top()))/CLOCKS_PER_SEC
            << "s" << std::endl;
  tictoc_stack.pop();
}

using namespace point_cloud_filtering;
using namespace point_cloud_meshing;

// Register the Histogram Type of fixed size. (for file saving only)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<135>,(float[135], histogram, histogram));

namespace object_localisation {

std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector;

void saveCloud(const sensor_msgs::PointCloud2& cloud){
  pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
  pcl::fromROSMsg(cloud, new_cloud);
  cloud_vector.push_back(new_cloud);
}


ObjectLocalisation::ObjectLocalisation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      preprocessed_size_(0),
      keypoints_size_(0),
      preprocessed_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB> ()),
      keypoint_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB> ()),
      normals_(new pcl::PointCloud<pcl::Normal>()),
      triangles_(new pcl::PolygonMesh()),
      cloud_vector_(),
      RoPS_histograms_(new pcl::PointCloud<pcl::Histogram <135> >()),
      RoPS_LRFs_(new pcl::PointCloud<pcl::ReferenceFrame>()),
      FPFH_signature_scene_(new pcl::PointCloud<pcl::FPFHSignature33>()),
      FPFH_signature_model_(new pcl::PointCloud<pcl::FPFHSignature33>()),
      model_scene_correspondence_(new pcl::Correspondences()),

      //Filtering Parameters
        number_of_average_clouds_ (15),
        number_of_median_clouds_ (5),
        z_threshold_ (0.005),
        planarSegmentationTolerance_ (0.02),
        min_number_of_inliers_ (30000),
        xmin_ (-0.5),
        xmax_ (0.5),
        ymin_ (-0.5),
        ymax_ (0.5),
        zmin_ (0.6),
        zmax_ (2),

      //Meshing Parameters
        number_of_neighbors_normal_ (30),
        max_edge_length_ (0.03),
        mu_ (1.5),
        max_nearest_neighbors_mesh_ (50),
        max_surface_angle_ (M_PI/4),
        min_angle_ (5*M_PI/180),
        max_angle_ (160*M_PI/180),
        normal_consistency_ (false),

      //Keypoint Detection Parameters
        normal_radius_ (0.008),
        salient_radius_ (0.015),
        border_radius_ (salient_radius_ *1.1),
        non_max_radius_ (salient_radius_*0.5),
        gamma_21_ (0.975),
        gamma_32_ (0.975),
        min_neighbors_ (5),
        threads_ (4),

      // Parameters for FPFH-Descriptor
        FPFH_radius_ (0.016),

      // Parameters for RoPS-Feature.
        RoPS_radius_ (0.03),
        crops_ (true),

      // Correspondence Search
        sqr_correspondence_distance_(50)
{
  // Timer on
  tic();

  // initializing publisher and subscriber
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, saveCloud);
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("object_detection/preprocessedCloud", 1, true);

  //Sample clouds
  ros::Rate r(60);
  while (cloud_vector.size() < number_of_average_clouds_ + number_of_median_clouds_)
  {
   ros::spinOnce();
   r.sleep();
  }
  cloud_vector_ = cloud_vector;

  std::cout << "Clouds are sampled."
            << "  width = " << cloud_vector[0].width
            << "  height = " << cloud_vector[0].height
            << "  size = " << cloud_vector[0].size() << std::endl;

  // Preprocessing (Filtering)
  preprocess();
  std::cout << "Cloud is filtered, segmented and clipped." << std::endl;

  // computing cloud resolution
//  computeCloudResolution(preprocessed_cloud_ptr_);
//  std::cout << "Cloud resolution is " << cloud_resolution_ << " meters." << std::endl;

  // computing normals
  computeNormals(preprocessed_cloud_ptr_);
  std::cout << "Normal are computed" << std::endl;

//  // Building Mesh
//  computeMesh(preprocessed_cloud_ptr_);
//  std::cout << "Mesh is calculated." << std::endl;

  // Keypoint Detection
  computeKeypoints(preprocessed_cloud_ptr_);
  std::cout << keypoints_size_ << " keypoints are detected." << std::endl;

//  // RoPS Descriptor Estimation
//  computeROPSDescriptor();
//  std::cout << RoPS_LRFs_->size() <<" RoPSfeatures deskriptors and LRF's are estimated." << std::endl;

  //Compute FPFH Signature
  computeFPFHDescriptor();
  std::cout << "FPFH signatures are calculated." << std::endl;

  //temporary
  //FPFH_signature_model_=FPFH_signature_scene_;
  loadFPFHSignature(1);
  std::cout << "loaded " << FPFH_signature_model_->size() << "signatures" << std::endl;

  //Correspondence Search
  FPFHcorrespondence();
  std::cout << model_scene_correspondence_->size()<< " correspondences are found." << std::endl;

  // publish filtered cloud
  publish(*preprocessed_cloud_ptr_);

  //Timer off
  toc();

  // saving files
  pcl::io::savePCDFileASCII ("PointCloud_preprocessed.pcd", *preprocessed_cloud_ptr_);
  pcl::io::savePCDFileASCII ("PointCloud_keypoints.pcd", *keypoint_cloud_ptr_);
  //pcl::io::savePCDFile  ("Object_Localisation_Files/PointCloud_RoPSHistograms.pcd", *histograms);
  //pcl::io::savePLYFile  ("Object_Localisation_Files/PointCloud_RoPSHistograms.ply", *histograms);
  //pcl::io::savePCDFile  ("Object_Localisation_Files/PointCloud_LRFs.pcd", *LRFs);
  //pcl::io::savePLYFile  ("Object_Localisation_Files/PointCloud_LRFs.ply", *LRFs);
  //pcl::io::savePLYFile ("Object_Localisation_Files/PointCloud_mesh.ply", *triangles);
  pcl::io::savePCDFile  ("FPFHSignature2.pcd", *FPFH_signature_scene_);
  pcl::io::savePLYFile  ("FPFHSignature2.ply", *FPFH_signature_scene_);
  std::cout << "Files are saved" << std::endl;

  // shut down node
  ros::requestShutdown();
}

ObjectLocalisation::~ObjectLocalisation()
{
}

void ObjectLocalisation::publish(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  pcl::PCLPointCloud2 cloud_pcl;
  pcl::toPCLPointCloud2(cloud, cloud_pcl);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(cloud_pcl, cloud_msg);

  // publishing cloud
  publisher_.publish(cloud_msg);
}

bool ObjectLocalisation::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl::isFinite((*cloud)[i]))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  cloud_resolution_ = res;
  return true;
  }

bool ObjectLocalisation::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setRadiusSearch(normal_radius_);
  n.compute (*normals_);
  return true;
}

bool ObjectLocalisation::preprocess(){

  //DO NOT MODIFY! Parameter recalculation
  std::vector<float> boundaries;
  boundaries.push_back(xmin_);
  boundaries.push_back(xmax_);
  boundaries.push_back(ymin_);
  boundaries.push_back(ymax_);
  boundaries.push_back(zmin_);
  boundaries.push_back(zmax_);

  filtering filtering;
  filtering.setNumberOfMedianClouds(number_of_median_clouds_);
  filtering.setNumberOfAverageClouds(number_of_average_clouds_);
  filtering.setInputClouds(cloud_vector_);
  filtering.setClippingBoundaries(boundaries);
  filtering.setZThreshold(z_threshold_);
  filtering.setPlanarSegmentationTolerance(planarSegmentationTolerance_);
  filtering.setMinNumberOfInliers(min_number_of_inliers_);
  filtering.compute(preprocessed_cloud_ptr_);
  unsigned int preprocessed_size_ = preprocessed_cloud_ptr_->size();
  return true;
}

bool ObjectLocalisation::computeKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
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
  iss_detector.setInputCloud (cloud);
  iss_detector.compute (*keypoint_cloud_ptr_);
  keypoints_size_ = keypoint_cloud_ptr_->size();

  return true;

}

bool ObjectLocalisation::computeMesh(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
    meshing meshing;
    meshing.setInputCloud(preprocessed_cloud_ptr_);
    meshing.setMaxAngle(max_angle_);
    meshing.setMinAngle(min_angle_);
    meshing.setMaxEdgeLength(max_edge_length_);
    meshing.setMu(mu_);
    meshing.setNumberOfNeighborsNormal(number_of_neighbors_normal_);
    meshing.setMaxSurfaceAngle(max_surface_angle_);
    meshing.setNormalConsistency(normal_consistency_);
    meshing.compute(*triangles_);
  return true;
}

bool ObjectLocalisation::computeROPSDescriptor(){
    unsigned int number_of_partition_bins = 5;
    unsigned int number_of_rotations = 3;

    std::vector<bool>* keypoints (new std::vector<bool> ());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZRGB>);
    search_method->setInputCloud (preprocessed_cloud_ptr_);

    pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator;
    feature_estimator.setSearchMethod (search_method);
    feature_estimator.setSearchSurface (preprocessed_cloud_ptr_);
    feature_estimator.setInputCloud (keypoint_cloud_ptr_);
    feature_estimator.setTriangles (triangles_->polygons);
    feature_estimator.setRadiusSearch (RoPS_radius_);
    feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
    feature_estimator.setNumberOfRotations (number_of_rotations);
    feature_estimator.setSupportRadius (RoPS_radius_);
    feature_estimator.setCrops (crops_);
    feature_estimator.compute(*RoPS_histograms_, *RoPS_LRFs_, *keypoints);

  return true;
}

bool ObjectLocalisation::computeFPFHDescriptor(){

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());

    // defining indices
    for (int k = 0; k < keypoints_size_; k++)
    {
      indices->indices.push_back (k);
    }

    fpfh.setSearchMethod(search_method);
    fpfh.setIndices(indices);
    fpfh.setInputCloud(keypoint_cloud_ptr_);
    fpfh.setSearchSurface(preprocessed_cloud_ptr_);
    fpfh.setInputNormals(normals_);
    fpfh.setRadiusSearch(FPFH_radius_);
    fpfh.compute (*FPFH_signature_scene_);

  return true;
}

bool ObjectLocalisation::FPFHcorrespondence(){

    pcl::KdTreeFLANN<pcl::FPFHSignature33 > match_search;

    match_search.setInputCloud(FPFH_signature_model_);

    for (size_t i = 0; i < FPFH_signature_scene_->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);

      if (!pcl::isFinite<pcl::FPFHSignature33> (FPFH_signature_scene_->at(i))) //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (FPFH_signature_scene_->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < sqr_correspondence_distance_) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_correspondence_->push_back (corr);
      }
    }
  return true;
}

bool ObjectLocalisation::loadFPFHSignature(int model_number){

  // load vertices

  std::ifstream FPFHSignature_file;
  std::string path = ros::package::getPath("object_localisation") + "/models/FPFHSignature_";
  std::stringstream number;
  number << model_number;
  path.append(number.str());
  std::string file_name = path + ".ply";
  FPFHSignature_file.open (file_name.c_str(), std::ifstream::in);
  std::cout << "loading:" << file_name << std::endl;

  for (std::string line; std::getline (FPFHSignature_file, line);)
    {
      pcl::FPFHSignature33 signature;
      std::istringstream in (line);
      unsigned int mark = 0;
      float sign =0;
      in >> mark;
      if (mark == 33)
      {
        for (int i=0; i<33; i++){
          in >> sign;
          signature.histogram[i] = sign;
        }
        FPFH_signature_model_->push_back (signature);
      }
    }
  return true;
}
} /* namespace object_loclisation*/
