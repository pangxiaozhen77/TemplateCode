#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <numeric>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/point_tests.h>

#include "rops_estimation.h"
//#include <pcl/features/rops_estimation.h>







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
  bool segment_plane1 = false;
  bool segment_plane2 = false;

  double orth_threshold1 = 0.05;            //threshold for plane suppression
  double orth_threshold2 = 0.05;

  //ISS keypoint detection
  bool calculate_borders1 = true;
  bool calculate_borders2 = true;

  double ISS_salient_radius1 = 15;           //!!!!!times model_resolution!!!!!!!!
  double ISS_salient_radius2 = 15;
  double ISS_non_max_radius1 = 5;
  double ISS_non_max_radius2 = 5;
  double normal_radius1 = 10;
  double normal_radius2 = 10;
  double border_radius1 = 25;
  double border_radius2 = 25;

  double iss_gamma_21_1 (0.975);
  double iss_gamma_32_1 (0.975);
  double iss_min_neighbors_1 (5);

  double iss_gamma_21_2 (0.975);
  double iss_gamma_32_2 (0.975);
  double iss_min_neighbors_2 (5);

  // Parameters for RoPS-Feature.
  float relative_radius1 = 25;
  bool crops1 = false;

  // Parameters for RoPS-Feature.
  float relative_radius2 = 25;
  bool crops2 = false;

  //correspondence

  double sqrt_dist_corr = 0.02;

  //clustering
  bool use_hough = false;
  double BinSize = 0.05;
  int Threshold = 1;

  //visualisation
  bool show_keypoints1 = true;
  bool show_keypoints2 = true;

  bool show_segmented_plane1 =false;
  bool show_segmented_plane2 =false;

  double sqrt_dist_overlay = 0.1;     //*model_resolution
  float x_shift = 0.2;

  //storage
  bool write_wo_false1 = false;       // save point cloud without points with z=0 to pcd
  bool write_wo_false2 = false;
  bool write_seg1 = false;            //save point cloud after segmentation to pcd
  bool write_seg2 = false;
  bool write_keypoints1 = true;      //save computed keypoints to pcd
  bool write_keypoints2 = true;



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
  std::vector<double> time_vec1;
  std::vector<double> time_vec2;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_org1) == -1)
  {
    if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[1], *cloud_org1) == -1)
    {
    PCL_ERROR ("Couldn't read input file \n");
    //return (-1);
    }
  }
  std::cout << "Loaded "
            << cloud_org1->height * cloud_org1->width
            << " data points from first input file "
            << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_org2) == -1)
  {
    if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[2], *cloud_org2) == -1)
    {
    PCL_ERROR ("Couldn't read input file \n");
    return (-1);
    }
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
    time_vec1.push_back(time);

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
    time_vec2.push_back(time);

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
    time_vec1.push_back(time);

    std::cout << "Plane segmented from cloud 1 leaving " << cloud_seg1->points.size()
                  << " points in " << time << " seconds" << std::endl;


    if (write_seg1)
    {
      pcl::io::savePLYFile ("cloud1_segmented.ply", *cloud_seg1);
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
      time_vec2.push_back(time);

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
  time_vec1.push_back(time);

  std::cout << "Resolution of cloud 1 is " << model_resolution1
            << " and was calculated in " << time << " seconds." << std::endl;

  //cloud 2

  tictoc_stack.push(clock());

  double model_resolution2;
  model_resolution2 = computeCloudResolution(cloud_seg2);

  time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
  tictoc_stack.pop();
  time_vec2.push_back(time);

  std::cout << "Resolution of cloud 2 is " << model_resolution2
            << " and was calculated in " << time << " seconds." << std::endl;

  //FIND ISS KEYPOINTS

  //cloud 1

  tictoc_stack.push(clock());

    double iss_normal_radius_1;
    double iss_border_radius_1;
    double iss_salient_radius_1;
    double iss_non_max_radius_1;

    int iss_threads_1 (4);

    // initialize clouds for keypoint detection

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());

    // set radii

    iss_salient_radius_1 = ISS_salient_radius1*model_resolution2;
    iss_non_max_radius_1 = ISS_non_max_radius1*model_resolution2;

    //
    // Compute keypoints
    //

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector1;

    iss_detector1.setSearchMethod (tree1);
    iss_detector1.setSalientRadius (iss_salient_radius_1);
    iss_detector1.setNonMaxRadius (iss_non_max_radius_1);

    if (calculate_borders1)
    {
      iss_normal_radius_1 = normal_radius1 * model_resolution2;
      iss_border_radius_1 = border_radius1 * model_resolution2;

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
    time_vec1.push_back(time);

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
      time_vec2.push_back(time);

      std::cout << "Found " << model_keypoints2->points.size() << " keypoints in cloud 2 in "
                << time << " seconds"<< std::endl;

      if (write_keypoints2)
      {
        pcl::io::savePCDFileASCII ("keypoints_cloud2.pcd", *model_keypoints2);
      }

 //MESHING
    //cloud1

      tictoc_stack.push(clock());


    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n1;
    pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treee11 (new pcl::search::KdTree<pcl::PointXYZ>);
    treee11->setInputCloud (cloud_seg1);
    n1.setInputCloud (cloud_seg1);
    n1.setSearchMethod (treee11);
    n1.setRadiusSearch (normal_radius1*model_resolution1);
    n1.compute (*normals1);

    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals1 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_seg1, *normals1, *cloud_with_normals1);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr treee21 (new pcl::search::KdTree<pcl::PointNormal>);
    treee21->setInputCloud (cloud_with_normals1);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp31;
    pcl::PolygonMesh triangles1;

    // Set the maximum distance between connected points (maximum edge length)
    gp31.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp31.setMu (1.5);
    gp31.setMaximumNearestNeighbors (50);
    gp31.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp31.setMinimumAngle(5*M_PI/180); // 10 degrees
    gp31.setMaximumAngle(160*M_PI/180); // 120 degrees
    gp31.setNormalConsistency(false);

    // Get result
    gp31.setInputCloud (cloud_with_normals1);
    gp31.setSearchMethod (treee21);
    gp31.reconstruct (triangles1);

    // Additional vertex information
    std::vector<int> parts1 = gp31.getPartIDs();
    std::vector<int> states1 = gp31.getPointStates();

    time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
          tictoc_stack.pop();

    time_vec1.push_back(time);

    std::cout << "Time elapsed meshing first cloud: "
              << time << std::endl;


    //cloud2

    tictoc_stack.push(clock());

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n2;
    pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treee2 (new pcl::search::KdTree<pcl::PointXYZ>);
    treee2->setInputCloud (cloud_seg2);
    n2.setInputCloud (cloud_seg2);
    n2.setSearchMethod (treee2);
    n2.setRadiusSearch (normal_radius2*model_resolution2);
    n2.compute (*normals2);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_seg2, *normals2, *cloud_with_normals2);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr treee22 (new pcl::search::KdTree<pcl::PointNormal>);
    treee22->setInputCloud (cloud_with_normals2);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp32;
    pcl::PolygonMesh triangles2;

    // Set the maximum distance between connected points (maximum edge length)
    gp32.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp32.setMu (1.5);
    gp32.setMaximumNearestNeighbors (50);
    gp32.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp32.setMinimumAngle(5*M_PI/180); // 10 degrees
    gp32.setMaximumAngle(160*M_PI/180); // 120 degrees
    gp32.setNormalConsistency(false);

    // Get result
    gp32.setInputCloud (cloud_with_normals2);
    gp32.setSearchMethod (treee22);
    gp32.reconstruct (triangles2);

    // Additional vertex information
    std::vector<int> parts2 = gp32.getPartIDs();
    std::vector<int> states2 = gp32.getPointStates();

    time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
          tictoc_stack.pop();

    time_vec2.push_back(time);

    std::cout << "Time elapsed meshing second cloud: " << time << std::endl;

    /// FROM PointXYZ to PointXYZRGB

    tictoc_stack.push(clock());


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_scene (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_model (new pcl::PointCloud<pcl::PointXYZRGB> ());

    scene->width = cloud_seg1->width;
    scene->height = cloud_seg1->height;
    scene->points.resize(scene->width*scene->height);

    keypoint_scene->width = model_keypoints1->width;
    keypoint_scene->height = model_keypoints1->height;
    keypoint_scene->points.resize(keypoint_scene->width*keypoint_scene->height);

    model->width = cloud_seg2->width;
    model->height = cloud_seg2->height;
    model->points.resize(model->width*model->height);

    keypoint_model->width = model_keypoints2->width;
    keypoint_model->height = model_keypoints2->height;
    keypoint_model->points.resize(keypoint_model->width*keypoint_model->height);


    for (int i = 0; i < scene->points.size(); i++)
    {
      scene->points[i].x = cloud_seg1->points[i].x;
      scene->points[i].y = cloud_seg1->points[i].y;
      scene->points[i].z = cloud_seg1->points[i].z;
      scene->points[i].r = 255;
      scene->points[i].g = 255;
      scene->points[i].b = 255;
    }

    for (int i = 0; i < keypoint_scene->points.size(); i++)
    {
      keypoint_scene->points[i].x = model_keypoints1->points[i].x;
      keypoint_scene->points[i].y = model_keypoints1->points[i].y;
      keypoint_scene->points[i].z = model_keypoints1->points[i].z;
      keypoint_scene->points[i].r = 255;
      keypoint_scene->points[i].g = 255;
      keypoint_scene->points[i].b = 255;
    }

    for (int i = 0; i < model->points.size(); i++)
    {
      model->points[i].x = cloud_seg2->points[i].x;
      model->points[i].y = cloud_seg2->points[i].y;
      model->points[i].z = cloud_seg2->points[i].z;
      model->points[i].r = 255;
      model->points[i].g = 255;
      model->points[i].b = 255;
    }

    for (int i = 0; i < keypoint_model->points.size(); i++)
    {
      keypoint_model->points[i].x = model_keypoints2->points[i].x;
      keypoint_model->points[i].y = model_keypoints2->points[i].y;
      keypoint_model->points[i].z = model_keypoints2->points[i].z;
      keypoint_model->points[i].r = 255;
      keypoint_model->points[i].g = 255;
      keypoint_model->points[i].b = 255;
    }

    time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
          tictoc_stack.pop();

    time_vec1.push_back(time/2);
    time_vec2.push_back(time/2);

    std::cout << "Time elapsed converting from XYZ to XYZRGB: "
              << time << std::endl;


    ///RoPS DESCRIPTOR

    //cloud1

    tictoc_stack.push(clock());


    // defining indices
    pcl::PointIndicesPtr indices1 = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
    for (int k = 0; k < model_keypoints1->points.size(); k++)
    {
      indices1->indices.push_back (k);
    }

    // Formating triangles
    std::vector <pcl::Vertices> vertices1;
    for (int i = 0; i < triangles1.polygons.size(); i++)
      {

        pcl::Vertices triangle;

          triangle.vertices.push_back(triangles1.polygons[i].vertices[0]);
          triangle.vertices.push_back(triangles1.polygons[i].vertices[1]);
          triangle.vertices.push_back(triangles1.polygons[i].vertices[2]);
          vertices1.push_back (triangle);
      }

    std::vector <pcl::Vertices> vertices2;
    for (int i = 0; i < triangles2.polygons.size(); i++)
      {
        pcl::Vertices triangle;

        triangle.vertices.push_back(triangles2.polygons[i].vertices[0]);
        triangle.vertices.push_back(triangles2.polygons[i].vertices[1]);
        triangle.vertices.push_back(triangles2.polygons[i].vertices[2]);
          vertices2.push_back (triangle);
      }


    //Dont Touch!! Parameters for RoPS. Histogram initialization and registration must be adjusted to changing sizes!!
    float support_radius1 = relative_radius1 * model_resolution2;
    unsigned int number_of_partition_bins1 = 5;
    unsigned int number_of_rotations1 = 3;

    // RoPS Feature Estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms1 (new pcl::PointCloud <pcl::Histogram <135> > ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr LRFs1 (new pcl::PointCloud <pcl::ReferenceFrame> ());
    std::vector<bool>* keypoints1 (new std::vector<bool> ());
    pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator1;

    search_method1->setInputCloud (scene);
    feature_estimator1.setSearchMethod (search_method1);
    feature_estimator1.setSearchSurface (scene);
    feature_estimator1.setInputCloud (keypoint_scene);
    feature_estimator1.setIndices (indices1);
    feature_estimator1.setTriangles (vertices2);
    feature_estimator1.setRadiusSearch (support_radius1);
    feature_estimator1.setNumberOfPartitionBins (number_of_partition_bins1);
    feature_estimator1.setNumberOfRotations (number_of_rotations1);
    feature_estimator1.setSupportRadius (support_radius1);
    feature_estimator1.setCrops (crops1);
    //feature_estimator1.compute(*histograms1);//, *LRFs1, *keypoints1);
    feature_estimator1.compute(*histograms1, *LRFs1, *keypoints1);
    std::cout << "mod!!!!" << std::endl;
    std::cout << histograms1->points[10].histogram[0] << std::endl;

    time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
          tictoc_stack.pop();

    time_vec1.push_back(time);

    std::cout << "Hist size: " << histograms1->size() << std::endl;
    std::cout << "LRFsize: " << LRFs1->size() << std::endl;
    std::cout << "Time elapsed calculating RoPS in the first cloud: "
              << time << std::endl;


    //cloud2

    tictoc_stack.push(clock());

    // defining indices
       pcl::PointIndicesPtr indices2 = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
       for (int k = 0; k < model_keypoints2->points.size(); k++)
         indices2->indices.push_back (k);


       //Dont Touch!! Parameters for RoPS. Histogram initialization and registration must be adjusted to changing sizes!!
       float support_radius2 = relative_radius2 * model_resolution2;
       unsigned int number_of_partition_bins2 = 5;
       unsigned int number_of_rotations2 = 3;


       // RoPS Feature Estimation
       pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
       pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms2 (new pcl::PointCloud <pcl::Histogram <135> > ());
       pcl::PointCloud<pcl::ReferenceFrame>::Ptr LRFs2 (new pcl::PointCloud <pcl::ReferenceFrame> ());
       std::vector<bool>* keypoints2 (new std::vector<bool> ());
       pcl::ROPSEstimation <pcl::PointXYZRGB, pcl::Histogram <135> > feature_estimator2;

       search_method2->setInputCloud (model);
       feature_estimator2.setSearchMethod (search_method2);
       feature_estimator2.setSearchSurface (model);
       feature_estimator2.setInputCloud (keypoint_model);
       feature_estimator2.setIndices (indices2);
       feature_estimator2.setTriangles (vertices2);
       feature_estimator2.setRadiusSearch (support_radius2);
       feature_estimator2.setNumberOfPartitionBins (number_of_partition_bins2);
       feature_estimator2.setNumberOfRotations (number_of_rotations2);
       feature_estimator2.setSupportRadius (support_radius2);
       feature_estimator2.setCrops (crops2);
       //feature_estimator2.compute(*histograms2);//, *LRFs2, *keypoints2);
       feature_estimator2.compute(*histograms2, *LRFs2, *keypoints2);
       time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
             tictoc_stack.pop();

       time_vec2.push_back(time);

       std::cout << "Hist size: " << histograms2->size() << std::endl;
       std::cout << "LRFsize: " << LRFs2->size() << std::endl;

       std::cout << "Time elapsed calculating RoPS in the second cloud: "
                 << time << std::endl;


       // compare Histograms of same point
       int index_kp1 = 0;
       int index_kp2 = 0;
       float maxz1 = -1;
       float maxz2 = -1;

       for (int index = 0; index < keypoint_scene->size(); index++)
       {
         if (keypoint_scene->points[index].z > maxz1)
         {
           maxz1 = keypoint_scene->points[index].z;
           index_kp1 = index;
         }
       }

       for (int index = 0; index < keypoint_model->size(); index++)
       {
         if (keypoint_model->points[index].z >= maxz2)
         {
           maxz2 = keypoint_model->points[index].z;
           index_kp2 = index;
         }
       }
       std::cout << "scene KP ind = " << index_kp1 << std::endl;
       std::cout << "model KP ind = " << index_kp2 << std::endl;
       std::cout << "Var 1" << std::endl;
       std::cout << "scene KP: x = " << keypoint_scene->points[index_kp1].x
                 << " y = " << keypoint_scene->points[index_kp1].y
                 << " z = " << keypoint_scene->points[index_kp1].z << std::endl;

       std::cout << "scene LRF: x = ("
                     << LRFs1->points[index_kp1].x_axis[0] << ", "
                     << LRFs1->points[index_kp1].x_axis[1] << ", "
                     << LRFs1->points[index_kp1].x_axis[2] << ")" << std::endl;
       std::cout << "scene LRF: y = ("
                     << LRFs1->points[index_kp1].y_axis[0] << ", "
                     << LRFs1->points[index_kp1].y_axis[1] << ", "
                     << LRFs1->points[index_kp1].y_axis[2] << ")" << std::endl;
       std::cout << "scene LRF: z = ("
                     << LRFs1->points[index_kp1].z_axis[0] << ", "
                     << LRFs1->points[index_kp1].z_axis[1] << ", "
                     << LRFs1->points[index_kp1].z_axis[2] << ")" << std::endl;

       std::cout << "model KP: x = " << keypoint_model->points[index_kp2].x
                 << " y = " << keypoint_model->points[index_kp2].y
                 << " z = " << keypoint_model->points[index_kp2].z << std::endl;

       std::cout << "model LRF: x = ("
                     << LRFs2->points[index_kp2].x_axis[0] << ", "
                     << LRFs2->points[index_kp2].x_axis[1] << ", "
                     << LRFs2->points[index_kp2].x_axis[2] << ")" << std::endl;
       std::cout << "model LRF: y = ("
                     << LRFs2->points[index_kp2].y_axis[0] << ", "
                     << LRFs2->points[index_kp2].y_axis[1] << ", "
                     << LRFs2->points[index_kp2].y_axis[2] << ")" << std::endl;
       std::cout << "model LRF: z = ("
                     << LRFs2->points[index_kp2].z_axis[0] << ", "
                     << LRFs2->points[index_kp2].z_axis[1] << ", "
                     << LRFs2->points[index_kp2].z_axis[2] << ")" << std::endl;

       std::cout << "scene Hist: ("
                     << histograms1->points[index_kp1].histogram[1] << ", "
                     << histograms1->points[index_kp1].histogram[2] << ", "
                     << histograms1->points[index_kp1].histogram[3] << ")" << std::endl;
       std::cout << "model Hist: ("
                     << histograms2->points[index_kp2].histogram[1] << ", "
                     << histograms2->points[index_kp2].histogram[2] << ", "
                     << histograms2->points[index_kp2].histogram[3] << ")" << std::endl;




       //
        //  Find Model-Scene Correspondences with KdTree
        //

       tictoc_stack.push(clock());

        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

        pcl::KdTreeFLANN<pcl::Histogram <135> > match_search;
        match_search.setInputCloud (histograms2);

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < histograms1->size (); ++i)
        {
          std::vector<int> neigh_indices (1);
          std::vector<float> neigh_sqr_dists (1);

          int found_neighs = match_search.nearestKSearch (histograms1->at (i), 1, neigh_indices, neigh_sqr_dists);
          if(found_neighs == 1 && neigh_sqr_dists[0] < sqrt_dist_corr) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
          {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
          }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;


        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;


        //  Clustering

        if(use_hough)
        {
          pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
          clusterer.setHoughBinSize (BinSize);
          clusterer.setHoughThreshold (Threshold);
          clusterer.setUseInterpolation (true);
          clusterer.setUseDistanceWeight (true);

          clusterer.setInputCloud (model_keypoints2);
          clusterer.setInputRf (LRFs2);
          clusterer.setSceneCloud (model_keypoints1);
          clusterer.setSceneRf (LRFs1);
          clusterer.setModelSceneCorrespondences (model_scene_corrs);

          //clusterer.cluster (clustered_corrs);
          clusterer.recognize (rototranslations, clustered_corrs);
        }
        else
        {
          pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
          gc_clusterer.setGCSize (BinSize);
          gc_clusterer.setGCThreshold (Threshold);

          gc_clusterer.setInputCloud (model_keypoints2);
          gc_clusterer.setSceneCloud (model_keypoints1);
          gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

          //gc_clusterer.cluster (clustered_corrs);
          gc_clusterer.recognize (rototranslations, clustered_corrs);
        }



        //
        //  Output results
        //
        double value = 0;
        int max;

        std::cout << "Model instances found: " << rototranslations.size () << std::endl;
        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
          if (value < clustered_corrs[i].size())
          {
            max = i;
            value = clustered_corrs[i].size();
          }

          std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
          std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;


          // Print the rotation matrix and translation vector
          Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
          Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

          printf ("\n");
          printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
          printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
          printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
          printf ("\n");
          printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
        }


        time =  ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
              tictoc_stack.pop();

        time_vec1.push_back(time);
        time_vec2.push_back(time);

        std::cout << "instance with most inliers is instance nr. " << max + 1 << std::endl;

        std::cout << "Correspondences found: " << model_scene_corrs->size ()
                  << ". Time elapsed: "<< time << std::endl << std::endl;

        std::cout << "Total time cloud 1: "
                  << std::accumulate(time_vec1.begin(), time_vec1.end(), 0.0) << std::endl;
        std::cout << "Total time cloud 2: "
                  << std::accumulate(time_vec2.begin(), time_vec2.end(), 0.0) << std::endl;


        //
        // VISUALIZATION
        // pre matching

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


        // post matching

        pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
        pcl::visualization::PCLVisualizer view_overlay ("overlay");
        viewer.addPointCloud (scene, "scene_cloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());


          //  We are translating the model so that it doesn't end in the middle of the scene representation
          pcl::transformPointCloud (*cloud_seg2, *off_scene_model, Eigen::Vector3f (x_shift,0,0), Eigen::Quaternionf (1, 0, 0, 0));
          pcl::transformPointCloud (*model_keypoints2, *off_scene_model_keypoints, Eigen::Vector3f (x_shift,0,0), Eigen::Quaternionf (1, 0, 0, 0));

          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
          viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");


          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoints_color_handler (model_keypoints1, 0, 0, 255);
          viewer.addPointCloud (model_keypoints1, scene_keypoints_color_handler, "scene_keypoints");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
          viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");



          pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZ> ());
          pcl::PointCloud<pcl::PointXYZ>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZ> ());
          pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ> ());
          pcl::transformPointCloud (*cloud_seg2, *rotated_model, rototranslations[max]);



          pcl::KdTreeFLANN<pcl::PointXYZ > distance;
          distance.setInputCloud (cloud_seg1);
          std::vector<float> sqrt_dist (1);;
          std::vector<int> index (1);
          for (int k = 0; k < rotated_model->points.size(); k++)
          {
            distance.nearestKSearch(rotated_model->points[k], 1, index, sqrt_dist);

            if (sqrt_dist[0] < sqrt_dist_overlay*model_resolution2)
            {
              inliers->push_back(rotated_model->points[k]);
            }
            else
            {
              outliers->push_back(rotated_model->points[k]);
            }
          }

          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outliers_color (outliers, 255, 0, 0);
          view_overlay.addPointCloud (outliers, outliers_color, "outliers");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inliers_color (inliers, 0, 255, 0);
          view_overlay.addPointCloud (inliers, inliers_color, "inliers");

            for (size_t j = 0; j < clustered_corrs[max].size (); ++j)
            {
              std::stringstream ss_line;
              ss_line << "correspondence_line" << max << "_" << j;
              pcl::PointXYZ& model_point = off_scene_model_keypoints->points [clustered_corrs[max][j].index_query];
              pcl::PointXYZ& scene_point = model_keypoints1->points[clustered_corrs[max][j].index_match];

              //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
              viewer.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_point, scene_point, 0, 255, 0, ss_line.str ());
            }


        while (!viewer.wasStopped () && !viewer1.wasStopped () && !viewer2.wasStopped () && !view_overlay.wasStopped ())
        {
          viewer.spinOnce ();
        }




  return (0);
}
