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


#ifndef _ROS_PAL_PCL_TRANSFORMS_H_
#define _ROS_PAL_PCL_TRANSFORMS_H_

// ROS headers
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// Eigen headers
#include <Eigen/Core>

namespace pal {

void convert(const Eigen::Matrix4d& transform,
             geometry_msgs::Pose& pose);

class TfTransformGetter {
public:

  /**
     * @brief TfTransformGetter
     */
  TfTransformGetter();

  /**
     * @brief ~TfTransformGetter
     */
  virtual ~TfTransformGetter();

  /**
     * @brief getTransform
     * @param childFrame
     * @param parentFrame
     * @param transform
     */
  void getTransform(const std::string& childFrame,
                    const std::string& parentFrame,
                    tf::Transform& transform);

  /**
     * @brief transformCloudToFrame
     * @param cloud
     * @param newFrameId
     * @param pointCloudInNewFrame
     */
  template <typename PointT>
  void transformCloudToFrame(const sensor_msgs::PointCloud2ConstPtr& cloud,
                             const std::string& newFrameId,
                             typename pcl::PointCloud<PointT>::Ptr& pointCloudInNewFrame);

protected:

  tf::TransformListener _tfListener;

};

template <typename PointT>
void TfTransformGetter::transformCloudToFrame(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                              const std::string& newFrameId,
                                              typename pcl::PointCloud<PointT>::Ptr& pointCloudInNewFrame)
{
  tf::Transform transform;
  getTransform(cloud->header.frame_id,
               newFrameId,
               transform);

  sensor_msgs::PointCloud2 cloudInNewFrame;
  pcl_ros::transformPointCloud(newFrameId, transform, *cloud, cloudInNewFrame);
  pcl::fromROSMsg(cloudInNewFrame, *pointCloudInNewFrame);
}


} //pal

#endif //_ROS_PAL_PCL_GEOMETRY_H_
