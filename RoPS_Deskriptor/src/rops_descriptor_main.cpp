//#include <pcl/features/rops_estimation.h>
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
  if (argc != 3)
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
  triangles_file.open (argv[2], std::ifstream::in);

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
  float mesh_resolution = 0.00176735;

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
