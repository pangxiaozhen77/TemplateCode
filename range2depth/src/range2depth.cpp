
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr rangeimage (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string scene_name  = argv[1];


  if (pcl::io::loadPLYFile<pcl::PointXYZ> ((scene_name + ".ply"), *rangeimage) == -1)
  {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
  }


  cloud->width = rangeimage->width;
  cloud->height = rangeimage->height;
  cloud->points.resize(cloud->width*cloud->height);

  for (int i = 0; i < cloud->size(); i++)
  {
    if(pcl::isFinite(rangeimage->points[i]))
    {
    cloud->points[i].x = rangeimage->points[i].x;
    cloud->points[i].y = rangeimage->points[i].y;
    cloud->points[i].z = rangeimage->points[i].z;
    }
    else
    {
      cloud->points[i].x = 0;
      cloud->points[i].y = 0;
      cloud->points[i].z = 0;
    }
  }

  pcl::io::savePCDFile ((scene_name + ".pcd"), *cloud);
}
