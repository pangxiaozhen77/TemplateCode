//#include <pcl/features/rops_estimation.h>
#include "rops_estimation.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <ctime>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>

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

double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
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
    //Considering the second neighbor since t, pcl::PointCloud<pcl::ReferenceFrame> &LRFs, std::vector<bool> &keypointshe first is the point itself.
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

int main (int argc, char** argv)
{
  //Getting input cloud name
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
  std::cout << std::endl << "Loaded " << size << " points." << std::endl;

  // Start timer
  tic();

  //Generating indices vector, could be replaced by an additional Keypoint Selection Method
  pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
  for (int k = 0; k < size; k++)
  {
    indices->indices.push_back (k);
  }

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

  // Compute model_resolution
  double model_resolution= computeCloudResolution(cloud);

//  //  Compute Normals
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
//  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
//  norm_est.setKSearch (10);
//  norm_est.setInputCloud (*cloud);
//  norm_est.compute (*cloud_normals);



  //Set Parameters for RoPSEstimation
  float support_radius = 0.0285f;
  float radius_search = support_radius;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  //Parameters for Keypoint Detection
  double iss_salient_radius_;
  double iss_non_max_radius_;
  double iss_border_radius_;
  iss_salient_radius_ = 8 * model_resolution;
  iss_non_max_radius_ = 6 * model_resolution;
  iss_border_radius_ =  12 * model_resolution;

  double iss_gamma_21_ (0.8);
  double iss_gamma_32_ (0.8);
  double iss_min_neighbors_ (5);
  int iss_threads_ (4);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZRGB>);
  search_method->setInputCloud (cloud);

  pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator;
  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr LRFs (new pcl::PointCloud <pcl::ReferenceFrame> ());
  std::vector<bool>* keypoints (new std::vector<bool> ());

  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud); //Whole PC
  feature_estimator.setInputCloud (cloud);  //Containing Keypoint PC or whole PC
  feature_estimator.setIndices (indices);   //containing Keypoints of PC (if only a part of Input Cloud should be used)
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);
  feature_estimator.setBorderRadius (iss_border_radius_);
  feature_estimator.setNonMaxRadius (iss_non_max_radius_);
  feature_estimator.setSalientRadius (iss_salient_radius_);
  feature_estimator.setThreshold21 (iss_gamma_21_);
  feature_estimator.setThreshold21 (iss_gamma_32_);
  feature_estimator.setMinNeighbors (iss_min_neighbors_);
  feature_estimator.setNormalRadius(iss_salient_radius_);
  feature_estimator.setNumberOfThreads (iss_threads_);
  feature_estimator.compute(*histograms, *LRFs, *keypoints);

  // End Timer
  toc();
  return (0);

  // Result Output
  for (int i=0; i< 10; i++){
  std::cout << "Histogram " << histograms->points[i].histogram[1] << std::endl;
  std::cout << "LRF " << LRFs->points[i].rf[1] << std::endl;
  }
  std::cout << "LRF size = " << LRFs->size() << std::endl;

  //TODO calculate the short descriptor, throwing out all non-keypoints from histograms

  // Save file
  //pcl::io::savePCDFileASCII  (scene_name + "_rops.pcd", *histograms);
  //std::cout << "Deskriptor is saved" << std::endl;
  //TODO Implement RGB feature_estimator using XYZ for LRF and local surface but RGB for descriptor
  //TODO Save Histogram and LRF


}
