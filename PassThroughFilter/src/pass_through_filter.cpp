#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

int
 main (int argc, char** argv)
{
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    std::cout << "\n\nUsage: pass_through_filter <PointCloud.pcd> <zmin>  <zmax>\n\n";
    return 0;
  }
  // Start timer
   time_t tstart, tend;
   tstart = time(0);

  // Load input file into a PointCloud<T> with an appropriate type
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

    //* the data should be available in cloud
    std::cout << "PCD is loaded" << std::endl;


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  float xmin = -0.27;
  float xmax = 0.5;
  pass.setFilterLimits (xmin, xmax);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  float ymin = -0.25;
  float ymax = 0.25;
  pass.setFilterLimits (ymin, ymax);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  float zmin = 0.7;
  float zmax = 1.0;
  pass.setFilterLimits (zmin, zmax);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);




  pcl::io::savePCDFileASCII ("clipedPCD.pcd", *cloud_filtered);
  std::cout << "PCD is cliped with:  xmin = " << xmin <<" and xmax = " << xmax << std::endl;
  std::cout << "                     ymin = " << ymin <<" and ymax = " << ymax << std::endl;
  std::cout << "                     zmin = " << zmin <<" and zmax = " << zmax << std::endl;

  // End Timer
  tend = time(0);
  std::cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< std::endl;

  return (0);
}
