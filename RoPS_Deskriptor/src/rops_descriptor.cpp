#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  if (argc != 4)
    return (-1);

  // load Point Cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZ> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read input file base \n");
      return (-1);
    }
  int height = cloud->height;
  int width = cloud->width;
  int size  = width*height;

  //load cloud XYZ and RGB in the corresponding subclouds
  pcl::PointCloud<pcl::PointXYZRGB> temp_cloud = *cloud;
  pcl::PointCloud<pcl::PointXYZ> temp_cloud_XYZ;
  pcl::PointCloud<pcl::PointXYZ> temp_cloud_RGB;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; i < width; i++)
    {
      pcl::PointXYZRGB point_cloud = temp_cloud(i,j);
      pcl::PointXYZ point_cloud_XYZ;
      pcl::PointXYZ point_cloud_RGB;

      point_cloud_XYZ.x = point_cloud.x;
      point_cloud_XYZ.y = point_cloud.y;
      point_cloud_XYZ.z = point_cloud.z;

      point_cloud_RGB.x = point_cloud.r;
      point_cloud_RGB.y = point_cloud.g;
      point_cloud_RGB.z = point_cloud.b;

      temp_cloud_XYZ(i,j) = point_cloud_XYZ;
      temp_cloud_RGB(i,j) = point_cloud_RGB;
    }
  }
  *cloud_XYZ = temp_cloud_XYZ;
  *cloud_RGB = temp_cloud_RGB;

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
      triangle.vertices.push_back (vertex - 1);
      in >> vertex;
      triangle.vertices.push_back (vertex - 1);
      in >> vertex;
      triangle.vertices.push_back (vertex - 1);
      triangles.push_back (triangle);
    }


  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
  search_method->setInputCloud (cloud_XYZ);

  pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud_XYZ);
  feature_estimator.setInputCloud (cloud_XYZ);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);

  //TODO Implement RGB feature_estimator using XYZ for LRF and local surface but RGB for descriptor
  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);

  return (0);
}
