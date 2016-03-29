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
  std::cout << "Time elapsed for filtering: "
            << ((double)(clock()-tictoc_stack.top()))/CLOCKS_PER_SEC
            << std::endl;
  tictoc_stack.pop();
}


int main (int argc, char** argv)
{

  std::string scene_name  = argv[1];
  std::cout << scene_name;

  // Start timer
  tic();

  if (argc != 4)
  {
    std::cout << "Wrong number of arguments!";
    return (-1);
  }
  // load Point Cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ((scene_name + ".pcd"), *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read input file base \n");
      return (-1);
    }

  //pcl:: PointXYZRGB point = cloud->operator ()(1,2);

  int height = cloud->height;
  int width = cloud->width;
  int size  = width * height;
  std::cout << "Loaded " << size << " points." << std::endl;

  //TODO load PKeypoint Indices, use other technique to determine keypoints
  pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
//  std::ifstream indices_file;
//  indices_file.open (argv[2], std::ifstream::in);
//  for (std::string line; std::getline (indices_file, line);)
//  {
//    std::istringstream in (line);
//    unsigne  std::cout << argv[3] << std::endl;d int index = 0;
//    in >> index;
//    indices->indices.push_back (index - 1);
//  }
//  indices_file.close ();
  for (int k = 0; k < size; k++)
  {
    indices->indices.push_back (k);
  }
  std::cout << "Using " << indices->indices.size() << " indices." << std::endl;

  // load vertices
  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);

  for (std::string line; std::getline (triangles_file, line);)
    {
      pcl::Vertices triangle;
      std::istringstream in (line);
      unsigned int vertex = 0;
      in >> vertex;
      // get vertex indices of Triangles
      if (vertex == 3)
      {
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);

        triangles.push_back (triangle);
      }
    }

  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZRGB>);
  search_method->setInputCloud (cloud);

  pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator;

  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  std::cout <<"support radius:  " <<feature_estimator.getSupportRadius()<<std::endl;
  feature_estimator.setSupportRadius (support_radius);

  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr LRFs (new pcl::PointCloud <pcl::ReferenceFrame> ());
  std::vector<bool>* keypoints (new std::vector<bool> ());

  feature_estimator.compute(*histograms, *LRFs, *keypoints);

  for (int i=0; i< size; i++){

  std::cout << "Histogram " << histograms->points[i].histogram[1] << std::endl;
  std::cout << "LRF size = " << LRFs->size() << std::endl;
  std::cout << "Keypoints size = " << keypoints->size() << std::endl;

  }
  //TODO calculate the short descriptor, throwing out all non-keypoints from histograms

  // Save file
  //pcl::io::savePCDFileASCII  (scene_name + "_rops.pcd", *histograms);
  //std::cout << "Deskriptor is saved" << std::endl;


  //TODO Implement RGB feature_estimator using XYZ for LRF and local surface but RGB for descriptor

  //TODO Save Histogram and LRF

  // End Timer
  toc();
  return (0);
}
