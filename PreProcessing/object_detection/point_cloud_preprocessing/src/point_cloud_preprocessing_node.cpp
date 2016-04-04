/*
 * point_cloud_preprocessing_node.cpp
 *
 *  Created on: Apr. 2, 2016
 *      Author: Yves Zimmermann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <point_cloud_preprocessing/PointCloudPreprocessing.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_preprocessing");
  ros::NodeHandle nodeHandle("~");
  point_cloud_preprocessing::PointCloudPreprocessing PointCloudPreprocessing(nodeHandle);
  ros::waitForShutdown();
  return 0;
}
