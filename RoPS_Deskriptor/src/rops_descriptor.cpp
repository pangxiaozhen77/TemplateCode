#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
int main (int argc, char** argv)
{
  // Start timer
   time_t tstart, tend;
   tstart = time(0);

  if (argc != 4)
    return (-1);

  // load Point Cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read input file base \n");
      return (-1);
    }

  int height = cloud->height;
  int width = cloud->width;
  int size  = width * height;

  //TODO load PKeypoint Indices, use other technique to determine keypoints
  pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
//  std::ifstream indices_file;
//  indices_file.open (argv[2], std::ifstream::in);
//  for (std::string line; std::getline (indices_file, line);)
//  {
//    std::istringstream in (line);
//    unsigned int index = 0;
//    in >> index;
//    indices->indices.push_back (index - 1);
//  }
//  indices_file.close ();
  for (int k = 0; k < size; k++)
  {
    indices->indices.push_back (k);
  }

  std::cout <<"Indices are generated." <<std::endl;

  // load vertices
  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);
  std::cout << argv[3] << std::endl;
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
  feature_estimator.setSupportRadius (support_radius);

  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);

  //TODO Implement RGB feature_estimator using XYZ for LRF and local surface but RGB for descriptor

  //TODO Save Histogram and LRF

  // End Timer
  tend = time(0);
  std::cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< std::endl;
  return (0);
}
