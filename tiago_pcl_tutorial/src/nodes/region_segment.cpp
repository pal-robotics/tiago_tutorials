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

/**
 * \author Job Van Dieten, Jordi Pages
 *
 * @brief This examples is the integration in TIAGo simulation of the pointclouds.org tutorial:
 *       pointclouds.org/documentation/tutorials/region_growing_segmentation.php
 */

// PAL
#include <tiago_pcl_tutorial/pcl_filters.hpp>
#include <tiago_pcl_tutorial/regionConfig.h>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

// PCL headers
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
// Needed for clang linking
// https://github.com/PointCloudLibrary/pcl/issues/2406
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>

// Std C++ headers
#include <iostream>
#include <vector>


namespace pal {

class RegionSegment
{
public:
  RegionSegment(ros::NodeHandle nh);
  ~RegionSegment();
  ros::Subscriber sub;
  ros::Publisher pub;

  bool _enable_downsampling;
  double _downsampling_leaf_size;
  int _min_neighbours;
  int _min_cluster_size, _max_cluster_size;
  double _smoothness_threshold, _curvature_threshold;

protected:

  void region_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

  void dynam_CB(const tiago_pcl_tutorial::regionConfig &config, uint32_t level);

  dynamic_reconfigure::Server<tiago_pcl_tutorial::regionConfig> reconf_server;
  dynamic_reconfigure::Server<tiago_pcl_tutorial::regionConfig>::CallbackType reconf_cb;
};


RegionSegment::RegionSegment(ros::NodeHandle nh):
  _enable_downsampling(true),
  _downsampling_leaf_size(0.01),
  _min_neighbours(30),
  _min_cluster_size(50),
  _max_cluster_size(1000000),
  _smoothness_threshold(3.0 / 180.0 * M_PI),
  _curvature_threshold(1.0)
{
  sub = nh.subscribe("/xtion/depth_registered/points", 1, &RegionSegment::cloud_cb, this);
  pub = nh.advertise<sensor_msgs::PointCloud2>("region", 1);

  reconf_cb = boost::bind(&RegionSegment::dynam_CB, this, _1, _2);
  reconf_server.setCallback(reconf_cb);
}

RegionSegment::~RegionSegment(){}

void RegionSegment::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  ROS_INFO_STREAM("Original number of points:               " << cloud->size());
  std::vector<int> removedNan;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, removedNan);
  ROS_INFO_STREAM("Number of points after removing NaNs:    " << cloud->size());

  if ( _enable_downsampling )
  {
    pal::downSample<pcl::PointXYZ>(cloud, cloud, _downsampling_leaf_size);
    ROS_INFO_STREAM("Number of points after downsampling:     " << cloud->size());
  }

  this->region_segment(cloud);
}

void RegionSegment::dynam_CB(const tiago_pcl_tutorial::regionConfig &config, uint32_t level)
{
  _enable_downsampling    = config.enableDownSampling;
  _downsampling_leaf_size = config.downSamplingLeafSize;
  _min_neighbours         = config.minNeighbours;
  _min_cluster_size       = config.minClusterSize;
  _max_cluster_size       = config.maxClusterSize;
  _smoothness_threshold   = config.smoothessThreshold/180.0 * M_PI;
  _curvature_threshold    = config.curvatureThreshold;
}

void RegionSegment::region_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.2);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (_min_cluster_size);
  reg.setMaxClusterSize (_max_cluster_size);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (_min_neighbours);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (_smoothness_threshold);
  reg.setCurvatureThreshold (_curvature_threshold);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  sensor_msgs::PointCloud2 cloud_out;

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  if ( !clusters.empty() )
  {
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::toROSMsg(*colored_cloud, cloud_out);
  }
  cloud_out.header.frame_id = cloud->header.frame_id;

  pub.publish(cloud_out);
}

} //pal


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segment_regions");
  ros::NodeHandle nh;

  pal::RegionSegment rs(nh);

  ros::spin();
  return 0;
}
