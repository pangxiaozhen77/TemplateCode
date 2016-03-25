/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_FEATURE_H_
#define PCL_FEATURES_IMPL_FEATURE_H_

#include <pcl/search/pcl_search.h>

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::Feature<PointInT, PointOutT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    if (surface_->isOrganized () && input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointInT> (false));
  }
  
  if (tree_->getInputCloud () != surface_) // Make sure the tree searches the surface
    tree_->setInputCloud (surface_); 


  // Do a fast check to see if the search parameters are well defined
  if (search_radius_ != 0.0)
  {
    if (k_ != 0)
    {
      PCL_ERROR ("[pcl::%s::compute] ", getClassName ().c_str ());
      PCL_ERROR ("Both radius (%f) and K (%d) defined! ", search_radius_, k_);
      PCL_ERROR ("Set one of them to zero first and then re-run compute ().\n");
      // Cleanup
      deinitCompute ();
      return (false);
    }
    else // Use the radiusSearch () function
    {
      search_parameter_ = search_radius_;
      // Declare the search locator definition
      int (KdTree::*radiusSearchSurface)(const PointCloudIn &cloud, int index, double radius,
                                         std::vector<int> &k_indices, std::vector<float> &k_distances,
                                         unsigned int max_nn) const = &pcl::search::Search<PointInT>::radiusSearch;
      search_method_surface_ = boost::bind (radiusSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5, 0);
    }
  }
  else
  {
    if (k_ != 0) // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      // Declare the search locator definition
      int (KdTree::*nearestKSearchSurface)(const PointCloudIn &cloud, int index, int k, std::vector<int> &k_indices,
                                           std::vector<float> &k_distances) const = &KdTree::nearestKSearch;
      search_method_surface_ = boost::bind (nearestKSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5);
    }
    else
    {
      PCL_ERROR ("[pcl::%s::compute] Neither radius nor K defined! ", getClassName ().c_str ());
      PCL_ERROR ("Set one of them to a positive number first and then re-run compute ().\n");
      // Cleanup
      deinitCompute ();
      return (false);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::Feature<PointInT, PointOutT>::deinitCompute ()
{
  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Feature<PointInT, PointOutT>::compute (PointCloudOut &output, pcl::PointCloud<pcl::ReferenceFrame> &LRFs)
{
  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());

  // Check if the output will be computed for all points or only a subset
  // If the input width or height are not set, set output width as size
  if (indices_->size () != input_->points.size () || input_->width * input_->height == 0)
  {
    output.width = static_cast<uint32_t> (indices_->size ());
    output.height = 1;
  }
  else
  {
    output.width = input_->width;
    output.height = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Perform the actual feature computation
  computeFeature (output, LRFs);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing normals was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] ", getClassName ().c_str ());
    PCL_ERROR ("The number of points in the input dataset (%u) differs from ", surface_->points.size ());
    PCL_ERROR ("the number of points in the dataset containing the normals (%u)!\n", normals_->points.size ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointLT, typename PointOutT> bool
pcl::FeatureFromLabels<PointInT, PointLT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!labels_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing labels was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (labels_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] The number of points in the input dataset differs from the number of points in the dataset containing the labels!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointRFT> bool
pcl::FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames (const size_t& indices_size,
                                                                                    const LRFEstimationPtr& lrf_estimation)
{
  if (frames_never_defined_)
    frames_.reset ();

  // Check if input frames are set
  if (!frames_)
  {
    if (!lrf_estimation)
    {
      PCL_ERROR ("[initLocalReferenceFrames] No input dataset containing reference frames was given!\n");
      return (false);
    } else
    {
      //PCL_WARN ("[initLocalReferenceFrames] No input dataset containing reference frames was given! Proceed using default\n");
      PointCloudLRFPtr default_frames (new PointCloudLRF());
      lrf_estimation->compute (*default_frames);
      frames_ = default_frames;
    }
  }

  // Check if the size of frames is the same as the size of the input cloud
  if (frames_->points.size () != indices_size)
  {
    if (!lrf_estimation)
    {
      PCL_ERROR ("[initLocalReferenceFrames] The number of points in the input dataset differs from the number of points in the dataset containing the reference frames!\n");
      return (false);
    } else
    {
      //PCL_WARN ("[initLocalReferenceFrames] The number of points in the input dataset differs from the number of points in the dataset containing the reference frames! Proceed using default\n");
      PointCloudLRFPtr default_frames (new PointCloudLRF());
      lrf_estimation->compute (*default_frames);
      frames_ = default_frames;
    }
  }

  return (true);
}

#endif  //#ifndef PCL_FEATURES_IMPL_FEATURE_H_

