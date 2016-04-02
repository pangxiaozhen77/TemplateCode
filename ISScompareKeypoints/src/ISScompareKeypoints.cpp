#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <ctime>
#include <pcl/common/projection_matrix.h>
#include <vector>




std::stack<clock_t> tictoc_stack;

double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl::isFinite((*cloud)[i]))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

int
main (int argc, char** argv)
{

  ////////////////////////////////////////////////////////////////////////////////////////
  // SET PARAMETERS

  //suppression of false measurements (z = 0)
  bool supp_false_measurements1 = true;
  bool supp_false_measurements2 = true;

  //plane segmentation
  bool segment_plane1 = true;
  bool segment_plane2 = true;

  double orth_threshold1 = 0.05;            //threshold for plane suppression
  double orth_threshold2 = 0.05;

  //ISS keypoint detection
  bool calculate_borders1 = true;
  bool calculate_borders2 = true;

  double ISS_salient_radius1 = 6;           //!!!!!times model_resolution!!!!!!!!
  double ISS_salient_radius2 = 6;
  double ISS_non_max_radius1 = 4;
  double ISS_non_max_radius2 = 4;
  double normal_radius1 = 4;
  double normal_radius2 = 4;
  double border_radius1 = 1;
  double border_radius2 = 1;

  //visualisation
  bool show_keypoints1 = true;
  bool show_keypoints2 = true;

  bool show_segmented_plane1 =false;
  bool show_segmented_plane2 =false;

  //storage
  bool write_wo_false1 = false;       // save point cloud without points with z=0 to pcd
  bool write_wo_false2 = false;
  bool write_seg1 = true;            //save point cloud after segmentation to pcd
  bool write_seg2 = false;
  bool write_keypoints1 = true;      //save computed keypoints to pcd
  bool write_keypoints2 = false;

  ////////////////////////////////////////////////////////////////////////////////////////


  //LOAD INPUT CLOUDS

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ point;
  double time;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_org1) == -1)
  {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_org1->height * cloud_org1->width
            << " data points from first input file "
            << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_org2) == -1)
  {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_org2->height * cloud_org2->width
            << " data points from second input file "
            << std::endl;

//ELIMINATION OF MEASUREMENTS WITH Z=0
  //cloud 1


  if (supp_false_measurements1)
  {
    tictoc_stack.push(clock());

    for (int i = 0; i < cloud_org1->points.size(); i++)
    {
      if (cloud_org1->points[i].z != 0)
      {
        point.x = cloud_org1->points[i].x;
        point.y = cloud_org1->points[i].y;
        point.z = cloud_org1->points[i].z;
        cloud1->points.push_back(point);
      }
    }
    cloud1->height = 1;
    cloud1->width = cloud1->points.size();
    cloud1->points.resize(cloud1->width*cloud1->height);

    time = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    tictoc_stack.pop();

    std::cout << "Cloud 1 was resized to "<< cloud1->points.size()
              << " points in "<< time << " seconds."<< std::endl;

    if (write_wo_false1)
    {
      pcl::io::savePCDFileASCII ("cloud1_without_false_measurements.pcd", *cloud1);
    }
  }
  else
  {
    cloud1 = cloud_org1;
  }

  //cloud 2
  if (supp_false_measurements2)
  {
    tictoc_stack.push(clock());

    for (int i = 0; i < cloud_org2->points.size(); i++)
    {
      if (cloud_org2->points[i].z != 0)
      {
        point.x = cloud_org2->points[i].x;
        point.y = cloud_org2->points[i].y;
        point.z = cloud_org2->points[i].z;
        cloud2->points.push_back(point);
      }
    }
    cloud2->height = 1;
    cloud2->width = cloud2->points.size();
    cloud2->points.resize(cloud2->width*cloud2->height);

    time = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    tictoc_stack.pop();

    std::cout << "Cloud 2 was resized to "<< cloud2->points.size()
              << " points in "<< time << " seconds."<< std::endl;

    if (write_wo_false2)
    {
      pcl::io::savePCDFileASCII ("cloud2_without_false_measurements.pcd", *cloud2);
    }
  }
  else
  {
    cloud2 = cloud_org2;
  }

// PLANE SEGMENTATION
  //cloud 1

  pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);

  if (segment_plane1)
  {
    tictoc_stack.push(clock());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg1;
    // Optional
    seg1.setOptimizeCoefficients (true);
    // Mandatory
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (orth_threshold1);

    seg1.setInputCloud (cloud1);
    seg1.segment (*inliers1, *coefficients1);

    if (inliers1->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset in cloud 1.");
      return (-1);
    }




    int j = 0;
    for (int i = 0; i < cloud1->points.size(); i++)
    {
        if (i != inliers1->indices[j])
        {
            point.x = cloud1->points[i].x;
            point.y = cloud1->points[i].y;
            point.z = cloud1->points[i].z;
            cloud_seg1->points.push_back(point);
        }
        else
        {
          j++;
        }
    }

    cloud_seg1->height = 1;
    cloud_seg1->width = cloud_seg1->points.size();
    cloud_seg1->points.resize(cloud_seg1->width*cloud_seg1->height);

    time = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    tictoc_stack.pop();

    std::cout << "Plane segmented from cloud 1 leaving " << cloud_seg1->points.size()
                  << " points in " << time << " seconds" << std::endl;

    if (write_seg1)
    {
      pcl::io::savePCDFileASCII ("cloud1_segmented.pcd", *cloud_seg1);
    }
  }
  else
  {
    cloud_seg1 = cloud1;
  }

  //cloud 2

  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);

    if (segment_plane2)
    {
      tictoc_stack.push(clock());

      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg2;
      // Optional
      seg2.setOptimizeCoefficients (true);
      // Mandatory
      seg2.setModelType (pcl::SACMODEL_PLANE);
      seg2.setMethodType (pcl::SAC_RANSAC);
      seg2.setDistanceThreshold (orth_threshold2);

      seg2.setInputCloud (cloud2);
      seg2.segment (*inliers2, *coefficients2);

      if (inliers2->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset in cloud 2.");
        return (-1);
      }




      int j = 0;
      for (int i = 0; i < cloud2->points.size(); i++)
      {
          if (i != inliers2->indices[j])
          {
              point.x = cloud2->points[i].x;
              point.y = cloud2->points[i].y;
              point.z = cloud2->points[i].z;
              cloud_seg2->points.push_back(point);
          }
          else
          {
            j++;
          }
      }

      cloud_seg2->height = 1;
      cloud_seg2->width = cloud_seg2->points.size();
      cloud_seg2->points.resize(cloud_seg2->width*cloud_seg2->height);

      time = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
      tictoc_stack.pop();

      std::cout << "Plane segmented from cloud 2 leaving " << cloud_seg2->points.size()
                    << " points in " << time << " seconds." << std::endl;

      if (write_seg2)
      {
        pcl::io::savePCDFileASCII ("cloud2_segmented.pcd", *cloud_seg2);
      }
    }
    else
    {
      cloud_seg2 = cloud2;
    }



  //COMPUTE MODEL RESOLUTION

  //cloud 1

  tictoc_stack.push(clock());

  double model_resolution1;
  model_resolution1 = computeCloudResolution(cloud_seg1);

  time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
  tictoc_stack.pop();

  std::cout << "Resolution of cloud 1 is " << model_resolution1
            << " and was calculated in " << time << " seconds." << std::endl;

  //cloud 2

  tictoc_stack.push(clock());

  double model_resolution2;
  model_resolution2 = computeCloudResolution(cloud_seg2);

  time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
  tictoc_stack.pop();

  std::cout << "Resolution of cloud 2 is " << model_resolution2
            << " and was calculated in " << time << " seconds." << std::endl;

  //FIND ISS KEYPOINTS

  //cloud 1

  tictoc_stack.push(clock());

    double iss_normal_radius_1;
    double iss_border_radius_1;
    double iss_salient_radius_1;
    double iss_non_max_radius_1;
    double iss_gamma_21_1 (0.975);
    double iss_gamma_32_1 (0.975);
    double iss_min_neighbors_1 (5);
    int iss_threads_1 (4);

    // initialize clouds for keypoint detection

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());

    // set radii

    iss_salient_radius_1 = ISS_salient_radius1*model_resolution1;
    iss_non_max_radius_1 = ISS_non_max_radius1*model_resolution1;

    //
    // Compute keypoints
    //

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector1;

    iss_detector1.setSearchMethod (tree1);
    iss_detector1.setSalientRadius (iss_salient_radius_1);
    iss_detector1.setNonMaxRadius (iss_non_max_radius_1);

    if (calculate_borders1)
    {
      iss_normal_radius_1 = normal_radius1 * model_resolution1;
      iss_border_radius_1 = border_radius1 * model_resolution1;

      iss_detector1.setNormalRadius (iss_normal_radius_1);
      iss_detector1.setBorderRadius (iss_border_radius_1);
    }

    iss_detector1.setThreshold21 (iss_gamma_21_1);
    iss_detector1.setThreshold32 (iss_gamma_32_1);
    iss_detector1.setMinNeighbors (iss_min_neighbors_1);
    iss_detector1.setNumberOfThreads (iss_threads_1);
    iss_detector1.setInputCloud (cloud_seg1);
    iss_detector1.compute (*model_keypoints1);

    time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    tictoc_stack.pop();

    std::cout << "Found " << model_keypoints1->points.size() << " keypoints in cloud 1 in "
              << time << " seconds"<< std::endl;

    if (write_keypoints1)
    {
      pcl::io::savePCDFileASCII ("keypoints_cloud1.pcd", *model_keypoints1);
    }
    //cloud 2

    tictoc_stack.push(clock());

      double iss_normal_radius_2;
      double iss_border_radius_2;
      double iss_salient_radius_2;
      double iss_non_max_radius_2;
      double iss_gamma_21_2 (0.975);
      double iss_gamma_32_2 (0.975);
      double iss_min_neighbors_2 (5);
      int iss_threads_2 (4);

      // initialize clouds for keypoint detection

      pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints2 (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());

      // set radii

      iss_salient_radius_2 = ISS_salient_radius2*model_resolution2;
      iss_non_max_radius_2 = ISS_non_max_radius2*model_resolution2;

      //
      // Compute keypoints
      //

      pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector2;

      iss_detector2.setSearchMethod (tree2);
      iss_detector2.setSalientRadius (iss_salient_radius_2);
      iss_detector2.setNonMaxRadius (iss_non_max_radius_2);

      if (calculate_borders2)
      {
        iss_normal_radius_2 = normal_radius2 * model_resolution2;
        iss_border_radius_2 = border_radius2 * model_resolution2;

        iss_detector2.setNormalRadius (iss_normal_radius_2);
        iss_detector2.setBorderRadius (iss_border_radius_2);
      }

      iss_detector2.setThreshold21 (iss_gamma_21_2);
      iss_detector2.setThreshold32 (iss_gamma_32_2);
      iss_detector2.setMinNeighbors (iss_min_neighbors_2);
      iss_detector2.setNumberOfThreads (iss_threads_2);
      iss_detector2.setInputCloud (cloud_seg2);
      iss_detector2.compute (*model_keypoints2);

      time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
      tictoc_stack.pop();

      std::cout << "Found " << model_keypoints2->points.size() << " keypoints in cloud 2 in "
                << time << " seconds"<< std::endl;

      if (write_keypoints2)
      {
        pcl::io::savePCDFileASCII ("keypoints_cloud2.pcd", *model_keypoints2);
      }


    //VISUALISATION

      pcl::ModelCoefficients plane_coeff;
      plane_coeff.values.resize (4);

    //cloud 1

    pcl::visualization::PCLVisualizer viewer1 ("Cloud1");
    viewer1.addPointCloud (cloud_seg1, "cloud1");
    viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

    if (show_keypoints1)
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(model_keypoints1, 0, 255, 0);
      viewer1.addPointCloud (model_keypoints1, single_color1, "keypoints1");
      viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints1");
    }

    if (show_segmented_plane1)
    {

      plane_coeff.values[0] = coefficients1->values[0];
      plane_coeff.values[1] = coefficients1->values[1];
      plane_coeff.values[2] = coefficients1->values[2];
      plane_coeff.values[3] = coefficients1->values[3];

      viewer1.addPlane (plane_coeff, "plane");
    }

    viewer1.addCoordinateSystem (1.0);
    viewer1.initCameraParameters ();

    //cloud 2

    pcl::visualization::PCLVisualizer viewer2 ("Cloud2");
    viewer2.addPointCloud (cloud_seg2, "cloud2");
    viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

    if (show_keypoints2)
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(model_keypoints2, 0, 255, 0);
      viewer2.addPointCloud (model_keypoints2, single_color2, "keypoints2");
      viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints2");
    }

    if (show_segmented_plane2)
    {
      plane_coeff.values[0] = coefficients2->values[0];
      plane_coeff.values[1] = coefficients2->values[1];
      plane_coeff.values[2] = coefficients2->values[2];
      plane_coeff.values[3] = coefficients2->values[3];

      viewer2.addPlane (plane_coeff, "plane");
    }

    viewer2.addCoordinateSystem (1.0);
    viewer2.initCameraParameters ();

    while (!viewer1.wasStopped () && !viewer2.wasStopped ())
    {
      viewer1.spinOnce (100);
      viewer2.spinOnce (100);
    }


  return (0);
}
