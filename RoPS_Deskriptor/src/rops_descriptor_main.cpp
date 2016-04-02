/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Yves Zimmermann
 * Email  : yvesz@ethz.ch
 *
 *
 * Give the path to the location and name stem of the scene, keypoint and mesh files
 * e.g. arg = "~/PathToClouds/Cloudname_unorganized"
 * the files have to be named the following way:
 *
 * SceneCloud:     Cloudname_unorganized.pcd
 * KeypointCloud:  Cloudname_unorganized_Keypoints.pcd
 * Mesh:           Cloudname_unorganized_Mesh.ply
 *
 */


#include "rops_estimation.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <ctime>
#include <pcl/io/ply_io.h>

std::stack<clock_t> tictoc_stack;


void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "Time elapsed for RoPS-Estimation: "
            << ((double)(clock()-tictoc_stack.top()))/CLOCKS_PER_SEC
            << std::endl;
  tictoc_stack.pop();
}

// Register the Histogram Type of fixed size. (for file saving only)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<135>,(float[135], histogram, histogram));

int main (int argc, char** argv)
{
  // Check arguments
  if (argc != 2)
  {
    std::cout << "Wrong number of arguments!";
    return (-1);
  }

  // load Point Cloud
  std::string scene_name  = argv[1];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ((scene_name + ".pcd"), *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read input file scene \n");
      return (-1);
    }
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ((scene_name + "_Keypoints.pcd"), *keypoint_cloud) == -1)
    {
      PCL_ERROR ("Couldn't read input file keypoints \n");
      return (-1);
    }

  // Get cloud sizes
  int height = cloud->height;
  int width = cloud->width;
  int size  = width * height;
  std::cout << std::endl << "Loaded " << size << " scene points." << std::endl;

  int heightk = keypoint_cloud->height;
  int widthk = keypoint_cloud->width;
  int sizek  = widthk * heightk;
  std::cout << "Loaded " << sizek << " keypoints." << std::endl;



  // defining indices
  pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
  for (int k = 0; k < sizek; k++)
    indices->indices.push_back (k);

  // load vertices
  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  std::string mesh_name  = scene_name + "_Mesh.ply";
  triangles_file.open (mesh_name.c_str(), std::ifstream::in);

  for (std::string line; std::getline (triangles_file, line);)
    {
      pcl::Vertices triangle;
      std::istringstream in (line);
      unsigned int vertex = 0;
      in >> vertex;
      if (vertex == 3)
      {
        in >> vertex;
        triangle.vertices.push_back (vertex);
        in >> vertex;
        triangle.vertices.push_back (vertex);
        in >> vertex;
        triangle.vertices.push_back (vertex);
        triangles.push_back (triangle);
      }
    }

  // Parameters for RoPS-Feature.
  float relative_radius = 25;
  bool crops = true;
  float mesh_resolution = 0.0017674;

  //Dont Touch!! Parameters for RoPS. Histogram initialization and registration must be adjusted to changing sizes!!
  float support_radius = relative_radius * mesh_resolution;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  std::cout << "Calculating c-RoPS features: " << crops << std::endl;
  std::cout << "support_radius = " << support_radius << std::endl;

  // Start timer
  tic();

  // RoPS Feature Estimation
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr LRFs (new pcl::PointCloud <pcl::ReferenceFrame> ());
  std::vector<bool>* keypoints (new std::vector<bool> ());
  pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator;

  search_method->setInputCloud (cloud);
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (keypoint_cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);
  feature_estimator.setCrops (crops);
  feature_estimator.compute(*histograms, *LRFs, *keypoints);

  // End Timer
  toc();

  // Save file
  pcl::io::savePCDFile  (scene_name + "_RoPSHistograms.pcd", *histograms);
  pcl::io::savePCDFile  (scene_name + "_LRFs.pcd", *LRFs);
  pcl::io::savePLYFile  (scene_name + "_RoPSHistograms.ply", *histograms);
  pcl::io::savePLYFile  (scene_name + "_LRFs.ply", *LRFs);

  return (0);
}
