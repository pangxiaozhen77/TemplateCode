#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include "gp3.h"
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <ctime>

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

int
main (int argc, char** argv)
{


  // Load input file into a PointCloud<T> with an appropriate type
   std::string scene = argv[1];
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

         if (pcl::io::loadPCDFile<pcl::PointXYZ> (scene + ".pcd", *cloud) == -1) //* load the file
         {
           PCL_ERROR ("Couldn't read input file \n");
           return (-1);
         }
         std::cout << "Loaded "
                   << cloud->width * cloud->height
                   << " data points from input file "
                   << std::endl;

  //* the data should be available in cloud
  std::cout << "PCD is loaded" << std::endl;

  // Start timer
  tic();

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (30);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures
  std::cout << "Normals are estimated" << std::endl;

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
  std::cout << "Normals are concatenated" << std::endl;

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (1.5);
  gp3.setMaximumNearestNeighbors (50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(5*M_PI/180); // 10 degrees
  gp3.setMaximumAngle(160*M_PI/180); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  std::cout << "Triangles are calculated" << std::endl;

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // End Timer
  toc();

  // Save file
  pcl::io::savePLYFile (scene + ".ply", triangles);
  std::cout << "Mesh is saved" << std::endl;



  // Finish
  return (0);
}
