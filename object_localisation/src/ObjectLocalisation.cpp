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
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/icp.h>


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

std::vector <pcl::PointCloud <PointType> > cloud_vector;

void saveCloud(const sensor_msgs::PointCloud2& cloud){
  pcl::PointCloud<PointType> new_cloud;
  pcl::fromROSMsg(cloud, new_cloud);
  cloud_vector.push_back(new_cloud);
}


ObjectLocalisation::ObjectLocalisation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      preprocessed_size_(0),
      keypoints_size_(0),
      preprocessed_cloud_ptr_(new pcl::PointCloud<PointType> ()),
      preprocessed_model_ptr_(new pcl::PointCloud<PointType> ()),
      transposed_model_ptr_(new pcl::PointCloud<PointType> ()),
      keypoint_cloud_ptr_(new pcl::PointCloud<PointType> ()),
      keypoint_model_ptr_(new pcl::PointCloud<PointType> ()),
      normals_(new pcl::PointCloud<pcl::Normal>()),
      triangles_(new pcl::PolygonMesh()),
      cloud_vector_(),
      RoPS_histograms_(new pcl::PointCloud<pcl::Histogram <135> >()),
      RoPS_LRFs_(new pcl::PointCloud<pcl::ReferenceFrame>()),
      FPFH_signature_scene_(new pcl::PointCloud<pcl::FPFHSignature33>()),
      FPFH_signature_model_(new pcl::PointCloud<pcl::FPFHSignature33>()),
      model_scene_correspondence_(new pcl::Correspondences()),
      FPFH_LRF_scene_(new pcl::PointCloud<pcl::ReferenceFrame>()),
      FPFH_LRF_model_(new pcl::PointCloud<pcl::ReferenceFrame>()),
      rototranslations_(),
      refined_rototranslations_(),
      clustered_correspondences_(),
      max_(0),
      model_index_(0),
      model_path_(),
      save_path_(),
      model_poses_(),

      //Models
        number_of_models_(1),

      //Filtering Parameters
        number_of_average_clouds_ (20),
        number_of_median_clouds_ (9),
        z_threshold_ (0.005),
        planarSegmentationTolerance_ (0.02),
        min_number_of_inliers_ (30000),
        xmin_ (-0.15),
        xmax_ (0.5),
        ymin_ (-0.25),
        ymax_ (0.25),
        zmin_ (0.6),
        zmax_ (1.05),

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
        salient_radius_ (0.012),
        border_radius_ (salient_radius_ *1.1),
        non_max_radius_ (salient_radius_*0.5),
        gamma_21_ (0.975),
        gamma_32_ (0.975),
        min_neighbors_ (5),
        threads_ (4),

      // Parameters for FPFH-Descriptor
        FPFH_radius_ (0.02),

      // Parameters for RoPS-Feature.
        RoPS_radius_ (0.03),
        crops_ (true),

      // Correspondence Search
        sqr_correspondence_distance_(70),

      // Local Reference Frame
        lrf_search_radius_ (0.01),

      // Hough clustering
        bin_size_ (0.05),
        threshold_ (1),

      // Refinement and Validation
        max_fitness_score_(0.00005),

      // Visualisation
        inlier_dist_(0.01),
        offset_(0.4)
{
  // Timer on
  tic();

  model_poses_.poses.resize(number_of_models_);
  refined_rototranslations_.resize(number_of_models_);

  std::string path = ros::package::getPath("object_localisation") ;
  model_path_ = path + "/models/";
  save_path_ = path + "/models/";
  std::cout << "Using Model Path: " << model_path_ << std::endl;
  std::cout << "Using Save Path: " << save_path_ << std::endl;

  // initializing publisher and subscriber
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, saveCloud);
  publisher_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("Pose", 1, true);

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
  std::cout << "Cloud is filtered, segmented and clipped. Using " << preprocessed_cloud_ptr_->size() << " points." << std::endl;

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
  std::cout << "FPFH signatures are computed." << std::endl;

  //Compute FPFH LRFs
  computeFPFHLRFs();
  std::cout << "FPFH LRFs are computed." << std::endl;

  // Search for different models
  for (model_index_ = 0; model_index_ < number_of_models_; model_index_++)
  {
    std::cout << "Searching for model " << model_index_ << " in scene." << std::endl;

    //Load Model Data
    loadModelData(model_index_);
    std::cout << "Loaded data for " << FPFH_signature_model_->size() << " model keypoints." << std::endl;

    //Correspondence Search
    FPFHcorrespondence();
    std::cout << model_scene_correspondence_->size()<< " correspondences are found." << std::endl;

    //Hough Clustering
    Clustering();
    int instances = rototranslations_.size ();
    std::cout << "Model instances found: " << instances << std::endl;

    //Refinement & Validation
    if (instances >= 1){
      Output();
      for (int i=0; i < rototranslations_.size(); i++){
        if (ICP(i)){
          continue;
        }else if (i == rototranslations_.size()-1 ){
          std::cout << "Found no translation with good enough match." << std::endl;
        }
      }
    }
  }

  // publish filtered cloud
  publisher_.publish(model_poses_);

  //Timer off
  toc();

  //Visualisation
  Visualisation();

  // saving files
  pcl::io::savePCDFileASCII (save_path_ + "Preprocessed_0.pcd", *preprocessed_cloud_ptr_);
  pcl::io::savePLYFile (save_path_ + "Preprocessed_0.ply", *preprocessed_cloud_ptr_);
  pcl::io::savePCDFileASCII (save_path_ + "Keypoints_0.pcd", *keypoint_cloud_ptr_);
  pcl::io::savePLYFile  (save_path_ + "LRFs_0.ply", *FPFH_LRF_scene_);
  pcl::io::savePLYFile  (save_path_ + "Signature_0.ply", *FPFH_signature_scene_);
  std::cout << "Files are saved in folder: " << save_path_ << std::endl;

  // shut down node
  ros::requestShutdown();
}

ObjectLocalisation::~ObjectLocalisation()
{
}

bool ObjectLocalisation::computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
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

bool ObjectLocalisation::computeNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  pcl::NormalEstimation<PointType, pcl::Normal> n;
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
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

bool ObjectLocalisation::computeKeypoints(const pcl::PointCloud<PointType>::ConstPtr &cloud){
  pcl::ISSKeypoint3D<PointType, PointType> iss_detector;
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

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

bool ObjectLocalisation::computeMesh(const pcl::PointCloud<PointType>::ConstPtr &cloud){
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
    pcl::search::KdTree<PointType>::Ptr search_method (new pcl::search::KdTree<PointType>);
    search_method->setInputCloud (preprocessed_cloud_ptr_);

    pcl::ROPSEstimation <PointType, pcl::Histogram <135> > feature_estimator;
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

    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<PointType>::Ptr search_method (new pcl::search::KdTree<PointType>);
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

bool ObjectLocalisation::computeFPFHLRFs(){
  pcl::BOARDLocalReferenceFrameEstimation<PointType, pcl::Normal, pcl::ReferenceFrame> rf_est;
          rf_est.setFindHoles (true);
          rf_est.setRadiusSearch (lrf_search_radius_);
          rf_est.setInputCloud (keypoint_cloud_ptr_);
          rf_est.setInputNormals (normals_);
          rf_est.setSearchSurface (preprocessed_cloud_ptr_);
          rf_est.compute (*FPFH_LRF_scene_);
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

bool ObjectLocalisation::loadModelData(int model_number){

  std::stringstream number;
  number << model_number;

  // load signatures
  std::ifstream FPFHSignature_file;
  std::string file_name = model_path_ + "Signature_";
  file_name.append(number.str());
  file_name = file_name + ".ply";
  FPFHSignature_file.open (file_name.c_str(), std::ifstream::in);

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

  // loading LRFs
  std::ifstream FPFH_LRF_file;
  std::string lrf_file_name = model_path_ + "LRFs_";
  lrf_file_name.append(number.str());
  lrf_file_name = lrf_file_name + ".ply";

  FPFH_LRF_file.open (lrf_file_name.c_str(), std::ifstream::in);

  for (std::string line; std::getline (FPFH_LRF_file, line);)
    {
    pcl::ReferenceFrame LRF;
      std::istringstream in (line);
      unsigned int mark = 0;
      float sign =0;
      in >> mark;
      if (mark == 3)
      {
        for (int i=0; i<3; i++){
          in >> sign;
          LRF.x_axis[i] = sign;
        }
        in >> mark;
        for (int i=0; i<3; i++){
          in >> sign;
          LRF.y_axis[i] = sign;
        }
        in >> mark;
        for (int i=0; i<3; i++){
          in >> sign;
          LRF.z_axis[i] = sign;
        }
        FPFH_LRF_model_->push_back (LRF);
      }
    }

  // loading kepoint cloud
  std::string keypoint_file_name = model_path_ + "Keypoints_";
  keypoint_file_name.append(number.str());
  keypoint_file_name = keypoint_file_name + ".pcd";

  if (pcl::io::loadPCDFile<PointType> (keypoint_file_name, *keypoint_model_ptr_) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read input file base \n");
      return (-1);
    }

  // loading preprocessed cloud
  std::string preprocessed_file_name = model_path_ + "Preprocessed_";
  preprocessed_file_name.append(number.str());
  preprocessed_file_name = preprocessed_file_name + ".pcd";

  if (pcl::io::loadPCDFile<PointType> (preprocessed_file_name, *preprocessed_model_ptr_) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read input file base \n");
      std::cout << preprocessed_model_ptr_->size() << std::endl;
      return (-1);
    }

  return true;
}

bool ObjectLocalisation::Clustering(){

        pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
        clusterer.setHoughBinSize (bin_size_);
        clusterer.setHoughThreshold (threshold_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (true);

        clusterer.setInputCloud (keypoint_model_ptr_);
        clusterer.setInputRf (FPFH_LRF_model_);
        clusterer.setSceneCloud (keypoint_cloud_ptr_);
        clusterer.setSceneRf (FPFH_LRF_scene_);
        clusterer.setModelSceneCorrespondences (model_scene_correspondence_);

        clusterer.cluster (clustered_correspondences_);
        clusterer.recognize (rototranslations_, clustered_correspondences_);

        return true;
}

bool ObjectLocalisation::ICP(int instance){

  pcl::transformPointCloud (*preprocessed_model_ptr_, *transposed_model_ptr_, rototranslations_[instance]);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(transposed_model_ptr_);
  icp.setInputTarget(preprocessed_cloud_ptr_);
  pcl::PointCloud<PointType> Final;
  icp.align(Final);
  *transposed_model_ptr_ = Final;

  if (icp.getFitnessScore() < max_fitness_score_){
    refined_rototranslations_[model_index_] = rototranslations_[instance]*icp.getFinalTransformation();
    max_ = instance;
    std::cout << "ICP has converged with fitness score: "  <<
    icp.getFitnessScore() << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = refined_rototranslations_[model_index_].block<3,3>(0, 0);
    Eigen::Vector3f translation = refined_rototranslations_[model_index_].block<3,1>(0, 3);

    model_poses_.poses[model_index_].position.x = translation[0];
    model_poses_.poses[model_index_].position.y = translation[1];
    model_poses_.poses[model_index_].position.z = translation[2];

    Eigen::Quaternionf quat(rotation);
    model_poses_.poses[model_index_].orientation.w =  quat.w();
    model_poses_.poses[model_index_].orientation.x =  quat.x();
    model_poses_.poses[model_index_].orientation.y =  quat.y();
    model_poses_.poses[model_index_].orientation.z =  quat.z();

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

    return true;
  }else{
    return false;
  }
}

bool ObjectLocalisation::Output(){

  double value = 0;
  for (int i = 0; i < rototranslations_.size (); ++i)
  {
    if (value < clustered_correspondences_[i].size())
    {
      max_ = i;
      value = clustered_correspondences_[i].size();
    }

    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_correspondences_[i].size () << std::endl;


    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations_[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations_[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
  std::cout << "Instance with most correspondences is instance nr. " << max_ + 1 << std::endl;

  return true;
}

bool ObjectLocalisation::Visualisation(){
  // post matching

  pcl::visualization::PCLVisualizer viewer ("Correspondence");
  pcl::visualization::PCLVisualizer view_overlay ("Overlay");
  viewer.addPointCloud<PointType> (preprocessed_cloud_ptr_, "scene_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (keypoint_cloud_ptr_, 0, 0, 255);
  viewer.addPointCloud (keypoint_cloud_ptr_, scene_keypoints_color_handler, "scene_keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

  //  We are translating the model so that it doesn't end in the middle of the scene representation
  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::transformPointCloud (*preprocessed_model_ptr_, *off_scene_model, Eigen::Vector3f (offset_,0,0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*keypoint_model_ptr_, *off_scene_model_keypoints, Eigen::Vector3f (offset_,0,0), Eigen::Quaternionf (1, 0, 0, 0));

  pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
  viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");

  pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
  viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");



  // Overlay visualisation

  pcl::PointCloud<PointType>::Ptr inliers (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr outliers (new pcl::PointCloud<PointType> ());

  pcl::KdTreeFLANN<PointType > distance;
  distance.setInputCloud (preprocessed_cloud_ptr_);

  std::vector<int> index (1);
  for (int k = 0; k < transposed_model_ptr_->points.size(); k++)
  {
    std::vector<float> sqrt_dist (1);
    distance.nearestKSearch(transposed_model_ptr_->points[k], 1, index, sqrt_dist);

    if (sqrt_dist[0] < inlier_dist_* inlier_dist_)
    {
      inliers->push_back(transposed_model_ptr_->points[k]);
    }
    else
    {
      outliers->push_back(transposed_model_ptr_->points[k]);
    }
  }
  std::cout << (float)inliers->size()/(inliers->size()+outliers->size())*100 << "% are inliers." << std::endl;

  pcl::visualization::PointCloudColorHandlerCustom<PointType> outliers_color (outliers, 255, 0, 0);
  view_overlay.addPointCloud (outliers, outliers_color, "outliers");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> inliers_color (inliers, 0, 255, 0);
  view_overlay.addPointCloud (inliers, inliers_color, "inliers");

    for (size_t j = 0; j < clustered_correspondences_[max_].size (); ++j)
    {
      std::stringstream ss_line;
      ss_line << "correspondence_line" << max_ << "_" << j;
      PointType& model_point = off_scene_model_keypoints->points [clustered_correspondences_[max_][j].index_query];
      PointType& scene_point = keypoint_cloud_ptr_->points[clustered_correspondences_[max_][j].index_match];

      //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
      viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
    }


  while (!viewer.wasStopped () && !view_overlay.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return true;
}
} /* namespace object_loclisation*/
