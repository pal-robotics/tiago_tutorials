/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/** \author Jordi Pages. */

// ROS headers
#include <pcl_ros/point_cloud.h>

// PCL headers
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#pragma GCC diagnostic warning "-Woverloaded-virtual"

namespace pal {

    /**
     * @brief downSample
     * @param inputCloud
     * @param outputCloud
     * @param leafSize
     */
    template <typename PointT>
    void downSample(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                    typename pcl::PointCloud<PointT>::Ptr& outputCloud,
                    double leafSize = 0.01);

    /**
     * @brief passThrough
     * @param inputCloud
     * @param axisName
     * @param limitMin
     * @param limitMax
     * @param outputCloud
     */
    template <typename PointT>
    void passThrough(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                     const std::string& axisName,
                     double limitMin,
                     double limitMax,
                     typename pcl::PointCloud<PointT>::Ptr& outputCloud);

    template <typename PointT>
    void downSample(const typename pcl::PointCloud<PointT>::Ptr &inputCloud,
                    typename pcl::PointCloud<PointT>::Ptr &outputCloud,
                    double leafSize);

    /**
     * @brief minNeighborsInRadiusFilter applies the pcl::RadiusOutlierRemoval filter to the given cloud
     * @param inputCloud
     * @param radius
     * @param minNeighbors
     * @param outputCloud
     */
    template <typename PointT>
    void minNeighborsInRadiusFilter(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                                    double radius,
                                    int minNeighbors,
                                    typename pcl::PointCloud<PointT>::Ptr& outputCloud);

    /**
     * @brief statisticalOutlierRemoval applies the pcl::StatisticalOutlierRemoval filter to the given cloud
     * @param inputCloud
     * @param numberOfPoints
     * @param stdDevMult
     * @param outputCloud
     */
    template <typename PointT>
    void statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                                   int numberOfPoints,
                                   double stdDevMult,
                                   typename pcl::PointCloud<PointT>::Ptr& outputCloud);


    /**
     * @brief extractIndices provides a new point cloud taking the points from the inputCloud specified
     *        by the given indices (setNegative == false) or those which are not in the indices
     *        (setNegative == true).
     * @param inputCloud
     * @param indices
     * @param setNegative
     * @param filteredCloud
     */
    template <typename PointT>
    void extractIndices(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                        const typename pcl::PointIndices::Ptr& indices,
                        bool setNegative,
                        typename pcl::PointCloud<PointT>::Ptr& filteredCloud);

    /**
     * @brief planeSegmentation given a point cloud locates the main plane and returns the point
     *        cloud of this main plane and its coefficients (A·X + B·Y + C·Z + D = 0).
     * @param inputCloud
     * @param planeCloud optional. Pointer to point cloud to store the points belonging to the plane
     * @param nonPlaneCloud optional. Pointer to point cloud to store those points not belonging to the plane
     * @param coefficients optional. Pointer to object to store plane coefficients
     * @return
     */
    template <typename PointT>
    bool planeSegmentation(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                           typename pcl::PointCloud<PointT>::Ptr* planeCloud,
                           typename pcl::PointCloud<PointT>::Ptr* nonPlaneCloud,
                           pcl::ModelCoefficients::Ptr* coefficients = NULL);

    /**
     * @brief removeMainPlane given a point cloud looks for the main plane and returns a new point cloud
     *        with the points lying on the main plane removed.
     * @param inputCloud
     * @param cloudMainPlaneRemoved
     * @return
     */
    template <typename PointT>
    bool removeMainPlane(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                         typename pcl::PointCloud<PointT>::Ptr& cloudMainPlaneRemoved);

    /**
     * @brief projectToPlane projects the input point cloud in a given plane and returns the resulting
     *        point cloud
     * @param inputCloud
     * @param planeCoefficients
     * @param outputCloud
     */
    template <typename PointT>
    void projectToPlane(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                        const pcl::ModelCoefficients::Ptr& planeCoefficients,
                        typename pcl::PointCloud<PointT>::Ptr& outputCloud);

    template <typename PointT>
    void estimateNormals(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                         int neighbors,
                         pcl::PointCloud<pcl::Normal>::Ptr* normals);

    /**
     * @brief cylinderSegmentation
     * @param inputCloud
     * @param cylinderCloud
     * @param neighbors number of neighbors considered to estimate the normals
     * @param cylinderCoefficients
     * @return
     */
    template <typename PointT>
    bool cylinderSegmentation(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                              typename pcl::PointCloud<PointT>::Ptr& cylinderCloud,
                              int neighbors,
                              double minRadius,
                              double maxRadius,
                              pcl::ModelCoefficients::Ptr& cylinderCoefficients);


    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template <typename PointT>
    void downSample(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                    typename pcl::PointCloud<PointT>::Ptr& outputCloud,
                    double leafSize)
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(inputCloud);
      vg.setLeafSize(leafSize, leafSize, leafSize);
      vg.setDownsampleAllData(true);
      vg.filter(*outputCloud);
    }

    template <typename PointT>
    void passThrough(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                     const std::string& axisName,
                     double limitMin,
                     double limitMax,
                     typename pcl::PointCloud<PointT>::Ptr& outputCloud)
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(inputCloud);
      pass.setFilterFieldName(axisName);
      pass.setFilterLimits(limitMin, limitMax);
      pass.filter(*outputCloud);
    }


    template <typename PointT>
    void minNeighborsInRadiusFilter(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                                    double radius,
                                    int minNeighbors,
                                    typename pcl::PointCloud<PointT>::Ptr& outputCloud)
    {
      pcl::RadiusOutlierRemoval<PointT> filter;
      filter.setInputCloud(inputCloud);
      filter.setRadiusSearch(radius);
      filter.setMinNeighborsInRadius(minNeighbors);
      filter.filter(*outputCloud);
    }

    template <typename PointT>
    void statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                                   int numberOfPoints,
                                   double stdDevMult,
                                   typename pcl::PointCloud<PointT>::Ptr& outputCloud)
    {
      pcl::StatisticalOutlierRemoval<PointT> filter;
      filter.setInputCloud(inputCloud);
      filter.setMeanK(numberOfPoints);
      filter.setStddevMulThresh(stdDevMult);
      filter.filter(*outputCloud);
    }

    template <typename PointT>
    void extractIndices(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                        const pcl::PointIndices::Ptr& indices,
                        bool setNegative,
                        typename pcl::PointCloud<PointT>::Ptr& filteredCloud)
    {
      pcl::ExtractIndices<PointT> extract;

      extract.setInputCloud(inputCloud);
      extract.setIndices(indices);
      extract.setNegative(setNegative);
      extract.filter(*filteredCloud);
    }

    template <typename PointT>
    bool locateMainPlane(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                   pcl::PointIndices::Ptr& inliers,
                   pcl::ModelCoefficients::Ptr& coefficients)
    {
      // Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.02);
      seg.setInputCloud(inputCloud);
      seg.segment(*inliers, *coefficients);

      return !inliers->indices.empty();
    }

    template <typename PointT>
    bool planeSegmentation(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                           typename pcl::PointCloud<PointT>::Ptr* planeCloud,
                           typename pcl::PointCloud<PointT>::Ptr* nonPlaneCloud,
                           pcl::ModelCoefficients::Ptr* coefficients)
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      bool planeFound = false;

      if ( coefficients == NULL )
      {
        pcl::ModelCoefficients::Ptr coeff( new pcl::ModelCoefficients() );
        planeFound = locateMainPlane<PointT>(inputCloud, inliers, coeff);
      }
      else
        planeFound = locateMainPlane<PointT>(inputCloud, inliers, *coefficients);

      if ( planeFound )
      {
        if ( planeCloud != NULL )
        {
          extractIndices<PointT>(inputCloud,
                                 inliers,
                                 false,       //points in the plane will survive
                                 *planeCloud);
        }
        if ( nonPlaneCloud != NULL )
        {
          extractIndices<PointT>(inputCloud,
                                 inliers,
                                 true,
                                 *nonPlaneCloud);
        }

        return true;
      }
      else
        return false;
    }

    template <typename PointT>
    bool removeMainPlane(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                         typename pcl::PointCloud<PointT>::Ptr& cloudMainPlaneRemoved)
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );

      if ( locateMainPlane(inputCloud, inliers, coefficients) )
      {
        extractIndices(inputCloud,
                       inliers,
                       true,        //points not in the plane will survive
                       cloudMainPlaneRemoved);
        return true;
      }
      else
        return false;
    }

    template <typename PointT>
    void projectToPlane(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                        const pcl::ModelCoefficients::Ptr& planeCoefficients,
                        typename pcl::PointCloud<PointT>::Ptr& outputCloud)
    {
       // Create the filtering object
       pcl::ProjectInliers<PointT> proj;
       proj.setModelType(pcl::SACMODEL_PLANE);
       proj.setInputCloud(inputCloud);
       proj.setModelCoefficients(planeCoefficients);
       proj.filter(*outputCloud);
    }

    template <typename PointT>
    void estimateNormals(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                         int neighbors,
                         pcl::PointCloud<pcl::Normal>::Ptr& normals)
    {
      typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      pcl::NormalEstimation<PointT, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (inputCloud);
      ne.setKSearch (neighbors);
      ne.compute(*normals);
    }

    template <typename PointT>
    bool cylinderSegmentation(const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
                              typename pcl::PointCloud<PointT>::Ptr& cylinderCloud,
                              int neighbors,
                              double minRadius,
                              double maxRadius,
                              pcl::ModelCoefficients::Ptr& cylinderCoefficients)
    {
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      estimateNormals<PointT>(inputCloud,
                              neighbors,
                              normals);

      if ( normals->size() < 20 )
        return false;

      pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight (0.1);
      seg.setMaxIterations(10000);
      seg.setDistanceThreshold(0.05);
      seg.setRadiusLimits(minRadius, maxRadius);
      seg.setInputCloud(inputCloud);
      seg.setInputNormals(normals);
      seg.segment(*inliers, *cylinderCoefficients);

      if ( !inliers->indices.empty() )
      {
        //Extract inliers
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cylinderCloud);
        return true;
      }
      else
        return false;
    }


} //pal
