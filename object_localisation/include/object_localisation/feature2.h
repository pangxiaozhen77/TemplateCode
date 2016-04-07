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

#ifndef PCL_FEATURE2_H_
#define PCL_FEATURE2_H_

#if defined __GNUC__
#  pragma GCC system_header 
#endif

#include <boost/function.hpp>
#include <boost/bind.hpp>
// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>

namespace pcl
{

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Feature represents the base feature class. Some generic 3D operations that
    * are applicable to all features are defined here as static methods.
    *
    * \attention
    * The convention for a feature descriptor is:
    *   - if the nearest neighbors for the query point at which the descriptor is to be computed cannot be
    *     determined, the descriptor values will be set to NaN (not a number)
    *   - it is impossible to estimate a feature descriptor for a point that doesn't have finite 3D coordinates.
    *     Therefore, any point that has NaN data on x, y, or z, will most likely have its descriptor set to NaN.
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT>
  class Feature2 : public PCLBase<PointInT>
  {
    public:
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;

      typedef PCLBase<PointInT> BaseClass;

      typedef boost::shared_ptr< Feature2<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr< const Feature2<PointInT, PointOutT> > ConstPtr;

      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      typedef boost::function<int (size_t, double, std::vector<int> &, std::vector<float> &)> SearchMethod;
      typedef boost::function<int (const PointCloudIn &cloud, size_t index, double, std::vector<int> &, std::vector<float> &)> SearchMethodSurface;

    public:
      /** \brief Empty constructor. */
      Feature2 () :
        feature_name_ (), search_method_surface_ (),
        surface_(), tree_(),
        search_parameter_(0), search_radius_(0), k_(0),
        fake_surface_(false)
      {}
            
      /** \brief Empty destructor */
      virtual ~Feature2 () {}

      /** \brief Provide a pointer to a dataset to add additional information
        * to estimate the features for every point in the input dataset.  This
        * is optional, if this is not set, it will only use the data in the
        * input cloud to estimate the features.  This is useful when you only
        * need to compute the features for a downsampled cloud.
        * \param[in] cloud a pointer to a PointCloud message
        */
      inline void
      setSearchSurface (const PointCloudInConstPtr &cloud)
      {
        surface_ = cloud;
        fake_surface_ = false;
        //use_surface_  = true;
      }

      /** \brief Get a pointer to the surface point cloud dataset. */
      inline PointCloudInConstPtr
      getSearchSurface () const
      {
        return (surface_);
      }

      /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () const
      {
        return (tree_);
      }

      /** \brief Get the internal search parameter. */
      inline double
      getSearchParameter () const
      {
        return (search_parameter_);
      }

      /** \brief Set the number of k nearest neighbors to use for the feature estimation.
        * \param[in] k the number of k-nearest neighbors
        */
      inline void
      setKSearch (int k) { k_ = k; }

      /** \brief get the number of k nearest neighbors used for the feature estimation. */
      inline int
      getKSearch () const
      {
        return (k_);
      }

      /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
        * estimation.
        * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
        */
      inline void
      setRadiusSearch (double radius)
      {
        search_radius_ = radius;
      }

      /** \brief Get the sphere radius used for determining the neighbors. */
      inline double
      getRadiusSearch () const
      {
        return (search_radius_);
      }

      /** \brief Base method for feature estimation for all points given in
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
        * and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut& output, pcl::PointCloud<pcl::ReferenceFrame>& LRFs, std::vector<bool>& keypoints);

    protected:
      /** \brief The feature name. */
      std::string feature_name_;

      /** \brief The search method template for points. */
      SearchMethodSurface search_method_surface_;

      /** \brief An input point cloud describing the surface that is to be used
        * for nearest neighbors estimation.
        */
      PointCloudInConstPtr surface_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The actual search parameter (from either \a search_radius_ or \a k_). */
      double search_parameter_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The number of K nearest neighbors to use for each point. */
      int k_;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string&
      getClassName () const { return (feature_name_); }

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief This method should get called after ending the actual computation. */
      virtual bool
      deinitCompute ();

      /** \brief If no surface is given, we use the input PointCloud as the surface. */
      bool fake_surface_;

      /** \brief Search for k-nearest neighbors using the spatial locator from
        * \a setSearchmethod, and the given surface from \a setSearchSurface.
        * \param[in] index the index of the query point
        * \param[in] parameter the search parameter (either k or radius)
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        *
        * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
        */
      inline int
      searchForNeighbors (size_t index, double parameter,
                          std::vector<int> &indices, std::vector<float> &distances) const
      {
        return (search_method_surface_ (*input_, index, parameter, indices, distances));
      }

      /** \brief Search for k-nearest neighbors using the spatial locator from
        * \a setSearchmethod, and the given surface from \a setSearchSurface.
        * \param[in] cloud the query point cloud
        * \param[in] index the index of the query point in \a cloud
        * \param[in] parameter the search parameter (either k or radius)
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        *
        * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
        */
      inline int
      searchForNeighbors (const PointCloudIn &cloud, size_t index, double parameter,
                          std::vector<int> &indices, std::vector<float> &distances) const
      {
        return (search_method_surface_ (cloud, index, parameter, indices, distances));
      }

    private:
      /** \brief Abstract feature estimation method.
        * \param[out] output the resultant features
        */
      virtual void
      computeFeature (PointCloudOut &output, pcl::PointCloud<pcl::ReferenceFrame> &LRFs, std::vector<bool> &keypoints) = 0;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#include "../src/feature2.hpp"

#endif  //#ifndef PCL_FEATURE_H_
