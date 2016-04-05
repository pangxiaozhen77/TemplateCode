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

#include "object_localisation/filtering.hpp"
#include "object_localisation/meshing.hpp"
#include "object_localisation/keypoint.hpp"
#include "object_localisation/ObjectLocalisation.hpp"

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
using namespace point_cloud_keypoint;

namespace object_localisation {

std::vector <pcl::PointCloud <pcl::PointXYZRGB> > cloud_vector;

void saveCloud(const sensor_msgs::PointCloud2& cloud){
  pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
  pcl::fromROSMsg(cloud, new_cloud);
  cloud_vector.push_back(new_cloud);
}


ObjectLocalisation::ObjectLocalisation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{

  //Filtering Parameters
    int number_of_average_clouds = 15;
    int number_of_median_clouds = 5;
    float z_threshold = 0.02;
    float planarSegmentationTolerance = 0.02;
    int min_number_of_inliers = 30000;
    float xmin = -0.15;
    float xmax = 0.5;
    float ymin = -0.5;
    float ymax = 0.5;
    float zmin = 0.5;
    float zmax = 2;
    ros::Rate r(60);

  //Meshing Parameters
    int number_of_neighbors_normal = 30;
    float max_edge_length = 0.03;
    float mu = 1.5;
    int max_nearest_neighbors_mesh = 50;
    float max_surface_angle = M_PI/4;
    float  min_angle = 5*M_PI/180;
    float  max_angle = 160*M_PI/180;
    bool  normal_consistency = false;

  //Keypoint Detection Parameters
    double normal_radius = 4;
    double border_radius = 4;
    double salient_radius = 6;
    double non_max_radius = 4;
    double gamma_21 = 0.975;
    double gamma_32 = 0.975;
    double min_neighbors = 5;
    int threads = 4;

  //Building boundary vector
  std::vector<float> boundaries;
  boundaries.push_back(xmin);
  boundaries.push_back(xmax);
  boundaries.push_back(ymin);
  boundaries.push_back(ymax);
  boundaries.push_back(zmin);
  boundaries.push_back(zmax);

  // Timer on
  tic();

  // initializing publisher and subscriber
  ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, saveCloud);
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("object_detection/preprocessedCloud", 1, true);

  //Sample clouds
  while (cloud_vector.size() < number_of_average_clouds + number_of_median_clouds)
  {
   ros::spinOnce();
   r.sleep();
  }

  std::cout << "Clouds are sampled."
            << "  width = " << cloud_vector[0].width
            << "  height = " << cloud_vector[0].height
            << "  size = " << cloud_vector[0].size() << std::endl;

  // Filtering
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB> ());
  filtering filtering;

  filtering.setNumberOfMedianClouds(number_of_median_clouds);
  filtering.setNumberOfAverageClouds(number_of_average_clouds);
  filtering.setInputClouds(cloud_vector);
  filtering.setClippingBoundaries(boundaries);
  filtering.setZThreshold(z_threshold);
  filtering.setPlanarSegmentationTolerance(planarSegmentationTolerance);
  filtering.setMinNumberOfInliers(min_number_of_inliers);
  filtering.compute(preprocessed_cloud_ptr);
  std::cout << "Cloud is filtered, segmented and clipped." << std::endl;

  // computing cloud resolution
  double model_resolution = computeCloudResolution(preprocessed_cloud_ptr);
  std::cout << "Model resolution is " << model_resolution << " meters." << std::endl;

  // Building Mesh
  pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);
  meshing meshing;
  meshing.setInputCloud(preprocessed_cloud_ptr);
  meshing.setMaxAngle(max_angle);
  meshing.setMinAngle(min_angle);
  meshing.setMaxEdgeLength(max_edge_length);
  meshing.setMu(mu);
  meshing.setNumberOfNeighborsNormal(number_of_neighbors_normal);
  meshing.setMaxSurfaceAngle(max_surface_angle);
  meshing.setNormalConsistency(normal_consistency);
  meshing.compute(*triangles);
  std::cout << "Mesh is calculated." << std::endl;

  // Keypoint Detection
  pcl::PointCloud<pcl::PointXYZRGB> keypoint_cloud;
  keypoint keypoint;
  keypoint.setModelResolution(model_resolution);
  keypoint.setSalientRadius (salient_radius);
  keypoint.setNonMaxRadius (non_max_radius);
  keypoint.setNormalRadius (normal_radius);
  keypoint.setBorderRadius (border_radius);
  keypoint.setGamma21 (gamma_21);
  keypoint.setGamma32 (gamma_32);
  keypoint.setMinNeighbors (min_neighbors);
  keypoint.setThreads (threads);
  keypoint.setInputCloud (preprocessed_cloud_ptr);
  keypoint.compute (keypoint_cloud);
  std::cout << keypoint_cloud.size() << " keypoints are detected." << std::endl;

  // publish filtered cloud
  publish(*preprocessed_cloud_ptr);

  //Timer off
  toc();

  // saving files
  pcl::io::savePCDFileASCII ("Object_Localisation_Files/PointCloud_preprocessed.pcd", *preprocessed_cloud_ptr);
  pcl::io::savePLYFile ("Object_Localisation_Files/PointCloud_mesh.ply", *triangles);
  pcl::io::savePCDFileASCII ("Object_Localisation_Files/PointCloud_keypoints.pcd", keypoint_cloud);
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

double ObjectLocalisation::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
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
  return res;
  }

} /* namespace object_loclisation*/
