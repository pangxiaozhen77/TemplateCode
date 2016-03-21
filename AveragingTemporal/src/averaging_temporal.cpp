/* \author Yves Zimmermann */


#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>


int
main (int argc, char** argv)
{
  // amount of clouds to average
  int nclouds = 5;

  // Load Base Cloud (including RGB)
  pcl::PointCloud<pcl::PointXYZRGB> base_cloud;
  std::string scene_name  = argv[1];

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (scene_name + "1.pcd", base_cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read input file base \n");
          return (-1);
        }
        std::cout << "Loaded "
                  << base_cloud.width * base_cloud.height
                  << " data points from input file "
                  << std::endl;
        std::cout << "width = " << base_cloud.width
                  << "    height = " << base_cloud.height
                  << std::endl;

  // Averaging

  for (int k=2; k<=nclouds; ++k)
  {
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    std::string temp_name = scene_name;
    std::stringstream k_act;
    k_act << k;
    temp_name.append(k_act.str());

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (temp_name + ".pcd", new_cloud) == -1) //* load the file
            {
              PCL_ERROR ("Couldn't read input file new\n");
              return (-1);
            }
    std::cout << "Loaded "
              << new_cloud.width * new_cloud.height
              << " data points from input file "
              << std::endl;
    std::cout << "width = " << new_cloud.width
              << "    height = " << new_cloud.height
              << std::endl;

    int i=240;
    int j=280;

    pcl::PointXYZRGB point_base_cloud = base_cloud(i,j);
    std::cout <<"Point content" << point_base_cloud << std::endl;

    pcl::PointXYZ point_new_cloud = new_cloud(i,j);
    std::cout <<"Point content" << point_new_cloud << std::endl;
  }

  // Finish
    return 0;
}
