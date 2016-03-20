#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/iss_3d.h>

int main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

      if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read input file \n");
        return (-1);
      }
      std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from input file "
                << std::endl;

      pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
      viewer.showCloud (cloud);
      while (!viewer.wasStopped ())
      {
      }

    return 0;
}
