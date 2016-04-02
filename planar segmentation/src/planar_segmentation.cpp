#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>

std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed segmenting plane: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->height * cloud->width
            << " data points from input file "
            << std::endl;

tic();
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);


  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  *seg_cloud = *cloud;

  for (int i = 0; i < inliers->indices.size(); i++)
  {
    seg_cloud->points[inliers->indices[i]].x = 0;
    seg_cloud->points[inliers->indices[i]].y = 0;
    seg_cloud->points[inliers->indices[i]].z = 0;
  }
toc();



  /*
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;
*/

  pcl::visualization::PCLVisualizer viewer ("Original Cloud");
  pcl::visualization::PCLVisualizer viewer_seg("Segmented Cloud");
  pcl::visualization::PCLVisualizer viewer_plane("Segmented Plane");


  pcl::ModelCoefficients plane_coeff;
  plane_coeff.values.resize (4);    // We need 4 values
  plane_coeff.values[0] = coefficients->values[0];
  plane_coeff.values[1] = coefficients->values[1];
  plane_coeff.values[2] = coefficients->values[2];
  plane_coeff.values[3] = coefficients->values[3];

  viewer.addPointCloud (cloud, "cloud");
  viewer_seg.addPointCloud (seg_cloud, "seg_cloud");
  viewer_plane.addPointCloud (seg_cloud, "seg_cloud");
  viewer_plane.addPlane (plane_coeff, "plane");


  while (!viewer.wasStopped () && !viewer_seg.wasStopped () && !viewer_plane.wasStopped ())
  {
    viewer.spinOnce (100);
  }


  return (0);
}
