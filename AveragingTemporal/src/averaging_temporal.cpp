/* \author Yves Zimmermann */


#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
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
  // Start timer
  tic();

  // amount of clouds to average
  int nclouds = 14;

  // Should RGB be averaged
  bool RGBaverage = true;

  // Clipping parameters
  float xmax = 0.2;
  float xmin = -0.2;

  float ymax = 0.20;
  float ymin = -0.25;

  float zmax = 0.85;
  float zmin = 0.6;


  // Load Base Cloud (including RGB)
  pcl::PointCloud<pcl::PointXYZRGB> base_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> unorganized_cloud;

  std::string scene_name  = argv[1];

  // Load Base Cloud (including RGB)
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

  // Start timer
  tic();

  // Averaging over ncloud clouds
  for (int k=2; k<=nclouds; ++k)
  {
    // adjust file path
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

          if (point_base_cloud.z != 0 && point_new_cloud.z != 0 && std::abs(point_base_cloud.z - point_new_cloud.z) < 0.02)
          {
            point_base_cloud.x = (point_base_cloud.x * (k-1) + point_new_cloud.x)/k;
            point_base_cloud.y = (point_base_cloud.y * (k-1) + point_new_cloud.y)/k;
            point_base_cloud.z = (point_base_cloud.z * (k-1) + point_new_cloud.z)/k;
            point_base_cloud.r = (point_base_cloud.r * (k-1) + point_new_cloud.r)/k;
            point_base_cloud.g = (point_base_cloud.g * (k-1) + point_new_cloud.g)/k;
            point_base_cloud.b = (point_base_cloud.b * (k-1) + point_new_cloud.b)/k;

          }
          else if (point_new_cloud.z != 0)
          {
            point_base_cloud.x = point_new_cloud.x;
            point_base_cloud.y = point_new_cloud.y;
            point_base_cloud.z = point_new_cloud.z;
            point_base_cloud.r = point_new_cloud.r;
            point_base_cloud.g = point_new_cloud.g;
            point_base_cloud.b = point_new_cloud.b;
          }

          base_cloud(i,j) = point_base_cloud;

          // clipping to unordered cloud
          bool ispartofcloud =  point_base_cloud.x >= xmin && point_base_cloud.x <= xmax &&
                                point_base_cloud.y >= ymin && point_base_cloud.y <= ymax &&
                                point_base_cloud.z >= zmin && point_base_cloud.z <= zmax;

          if (k == nclouds && ispartofcloud)
          {
            unorganized_cloud.push_back(point_base_cloud);
          }

        }
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

          if (point_base_cloud.z != 0 && point_new_cloud.z != 0 && std::abs(point_base_cloud.z - point_new_cloud.z) < 0.02)
          {
            point_base_cloud.x = (point_base_cloud.x * (k-1) + point_new_cloud.x)/k;
            point_base_cloud.y = (point_base_cloud.y * (k-1) + point_new_cloud.y)/k;
            point_base_cloud.z = (point_base_cloud.z * (k-1) + point_new_cloud.z)/k;
          }
          else if (point_new_cloud.z != 0)
          {
            point_base_cloud.x = point_new_cloud.x;
            point_base_cloud.y = point_new_cloud.y;
            point_base_cloud.z = point_new_cloud.z;
          }
            base_cloud(i,j) = point_base_cloud;

          // clipping to unordered cloud
          bool ispartofcloud =   point_base_cloud.x >= xmin && point_base_cloud.x <= xmax &&
                                 point_base_cloud.y >= ymin && point_base_cloud.y <= ymax &&
                                 point_base_cloud.z >= zmin && point_base_cloud.z <= zmax;

          if (k == nclouds && ispartofcloud)
          {
            unorganized_cloud.push_back(point_base_cloud);
          }
        }
      }
    }
  }

  // saving orgnanised clipped file


  // End Timer
  toc();

  // saving organised full cloud
  pcl::io::savePCDFileASCII (scene_name + "averaged.pcd", base_cloud);

  // saving unorganized clipped cloud
  pcl::io::savePCDFileASCII (scene_name + "averaged_unorganized.pcd", unorganized_cloud);


  // Finish
    return 0;
}
