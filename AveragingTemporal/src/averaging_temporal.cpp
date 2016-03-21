/* \author Yves Zimmermann */


#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <ctime>


int
main (int argc, char** argv)
{
  // Start timer
   time_t tstart, tend;
   tstart = time(0);

  // amount of clouds to average
  int nclouds = 20;
  bool RGBaverage = true;

  // Load Base Cloud (including RGB)
  pcl::PointCloud<pcl::PointXYZRGB> base_cloud;
  std::string scene_name  = argv[1];

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (scene_name + "1.pcd", base_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read input file base \n");
      return (-1);
    }

  int height = base_cloud.height;
  int width = base_cloud.width;

  std::cout << "Loaded " << width * height << " data points from input file. " << std::endl;
  std::cout << "width = " << width << "    height = " << height << std::endl;
  std::cout << "Averaging over " << nclouds << " clouds.  RGB average: " << RGBaverage << std::endl;

  // Averaging over ncloud clouds
  for (int k=2; k<=nclouds; ++k)
  {
    // adjust filepath
    std::string temp_name = scene_name;
    std::stringstream k_act;
    k_act << k;
    temp_name.append(k_act.str());

    // XYZ and RGB averaging
    if (RGBaverage)
    {
      //load file
      pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (temp_name + ".pcd", new_cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read input file \n");
          std::cout << temp_name << ".pcd" << std::endl;
          return (-1);
        }

      //iterate over points
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          pcl::PointXYZRGB point_base_cloud = base_cloud(i,j);
          pcl::PointXYZRGB point_new_cloud = new_cloud(i,j);

          if (point_base_cloud.z != 0 && point_new_cloud.z != 0)
          {
            point_base_cloud.x = (point_base_cloud.x * (k-1) + point_new_cloud.x)/k;
            point_base_cloud.y = (point_base_cloud.y * (k-1) + point_new_cloud.y)/k;
            point_base_cloud.z = (point_base_cloud.z * (k-1) + point_new_cloud.z)/k;
            point_base_cloud.r = (point_base_cloud.r * (k-1) + point_new_cloud.r)/k;
            point_base_cloud.g = (point_base_cloud.g * (k-1) + point_new_cloud.g)/k;
            point_base_cloud.b = (point_base_cloud.b * (k-1) + point_new_cloud.b)/k;

          }else if (point_new_cloud.z != 0)
          {
            point_base_cloud.x = point_new_cloud.x;
            point_base_cloud.y = point_new_cloud.y;
            point_base_cloud.z = point_new_cloud.z;
            point_base_cloud.r = point_new_cloud.r;
            point_base_cloud.g = point_new_cloud.g;
            point_base_cloud.b = point_new_cloud.b;
          }
          base_cloud(i,j) = point_base_cloud;
        }
          //std::cout << "Point (" << i << "," << j <<") :" << base_cloud(i,j) << std::endl;
      }
    }

    // XYZ averaging
    else
    {
      //load file
      pcl::PointCloud<pcl::PointXYZ> new_cloud;
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (temp_name + ".pcd", new_cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read input file \n");
          std::cout << temp_name << ".pcd" << std::endl;
          return (-1);
        }
      // iterating over points
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          pcl::PointXYZRGB point_base_cloud = base_cloud(i,j);
          pcl::PointXYZ point_new_cloud = new_cloud(i,j);

          if (point_base_cloud.x != 0 || point_base_cloud.y != 0 || point_base_cloud.y != 0)
          {
            point_base_cloud.x = (point_base_cloud.x * (k-1) + point_new_cloud.x)/k;
            point_base_cloud.y = (point_base_cloud.y * (k-1) + point_new_cloud.y)/k;
            point_base_cloud.z = (point_base_cloud.z * (k-1) + point_new_cloud.z)/k;

            base_cloud(i,j) = point_base_cloud;
          }
          //std::cout << "Point (" << i << "," << j <<") :" << base_cloud(i,j) << std::endl;
        }
      }
    }
  }

  // End Timer
  tend = time(0);
  std::cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< std::endl;

  // saving file
  pcl::io::savePCDFileASCII (scene_name + "averaged.pcd", base_cloud);

  // Finish
    return 0;
}
