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

// PAL headers
#include <tiago_pcl_tutorial/pcl_filters.hpp>
#include <tiago_pcl_tutorial/geometry.h>
#include <tiago_pcl_tutorial/tf_transforms.hpp>

// PCL headers
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
// Needed for clang linking
// https://github.com/PointCloudLibrary/pcl/issues/2406
#include <pcl/search/impl/search.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

// Eigen headers
#include <Eigen/Core>

namespace pal {

class CylinderDetector {

public:

  CylinderDetector(ros::NodeHandle& nh,
                   ros::NodeHandle& pnh);

  virtual ~CylinderDetector();

  void run();

protected:

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
               const Eigen::Matrix4d& transform,
               double cylinderHeight,
               const std_msgs::Header& header);

  void publishPose(const geometry_msgs::Pose& pose,
                   const std_msgs::Header& header);

  void print(const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
             int numberOfPoints);

  /**
   * @brief getPointAndVector Given a point cloud in which a cylinder has been fit, it computes
   *        the main axis of the cylinder and provides the centroid of the point cloud projected on the
   *        main axis and the main axis director vector
   * @param cylinderCloud
   * @param linePoint
   * @param lineVector
   */
  void getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                         Eigen::Vector3d& linePoint,
                         Eigen::Vector3d& lineVector);

  /**
   * @brief computeHeight calculate height of cylinder provided its point cloud and the main axis vector
   *        and the cylinder centroid.
   * @param cylinderCloud
   * @param centroid
   * @param mainAxis
   * @return
   */
  double computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                       const Eigen::Vector3d& centroid,
                       const Eigen::Vector3d& mainAxis);

  void projectPointToLine(const Eigen::Vector3d& linePoint,
                          const Eigen::Vector3d& lineVector,
                          const Eigen::Vector3d& pointToProject,
                          Eigen::Vector3d& projectedPoint);

  void start();
  void stop();

  ros::NodeHandle& _nh, _pnh;
  ros::CallbackQueue _cbQueue;
  bool _enabled;
  double _rate;

  // ROS interfaces
  ros::Subscriber _cloudSub;
  ros::Publisher  _cylinderCloudPub;
  ros::Publisher  _cylinderPosePub;
  ros::Publisher  _cylinderMarkerPub;
  ros::Publisher  _objectPub;
};


CylinderDetector::CylinderDetector(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh):
  _nh(nh),
  _pnh(pnh),
  _enabled(false),
  _rate(5.0)
{
  _nh.setCallbackQueue(&_cbQueue);

  pnh.param<double>("rate", _rate, _rate);

  _cylinderCloudPub  = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cylinder_cloud", 1);
  _cylinderPosePub   = _pnh.advertise< geometry_msgs::PoseStamped >("cylinder_pose", 1);
  _cylinderMarkerPub = _pnh.advertise<visualization_msgs::Marker>( "marker", 1 );
  _objectPub         = _pnh.advertise<object_recognition_msgs::RecognizedObjectArray>("recognized_objects",1);
}

CylinderDetector::~CylinderDetector()
{

}

void CylinderDetector::print(const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                             int numberOfPoints)
{
  std::stringstream ss;
  ss << std::endl << "Cylinder found with " << numberOfPoints << " points:  " << std::endl;
  ss << "\tRadius:      " << cylinderCoefficients->values[6] << " m" << std::endl;
  ss << "\tPoint:       (" <<
        cylinderCoefficients->values[0] << ", " <<
        cylinderCoefficients->values[1] << ", " <<
        cylinderCoefficients->values[2] << ")" << std::endl;
  ss << "\tAxis:        (" <<
        cylinderCoefficients->values[3] << ", " <<
        cylinderCoefficients->values[4] << ", " <<
        cylinderCoefficients->values[5] << ")" << std::endl;

  ROS_INFO_STREAM(ss.str());
}

void CylinderDetector::projectPointToLine(const Eigen::Vector3d& linePoint,
                                          const Eigen::Vector3d& lineVector,
                                          const Eigen::Vector3d& pointToProject,
                                          Eigen::Vector3d& projectedPoint)
{
  Eigen::Vector3d pointToLineVector;
  pointToLineVector = pointToProject - linePoint;
  projectedPoint = linePoint + (pointToLineVector.dot(lineVector) / lineVector.dot(lineVector) ) * lineVector;
}

void CylinderDetector::getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                                         Eigen::Vector3d& linePoint,
                                         Eigen::Vector3d& lineVector)
{
  //compute cylinder centroid
  Eigen::Vector4d centroid;
  centroid.setZero();
  pcl::compute3DCentroid<pcl::PointXYZRGB>(*cylinderCloud, centroid);

  linePoint(0,0) = cylinderCoefficients->values[0];
  linePoint(1,0) = cylinderCoefficients->values[1];
  linePoint(2,0) = cylinderCoefficients->values[2];

  lineVector(0,0) = cylinderCoefficients->values[3];
  lineVector(1,0) = cylinderCoefficients->values[4];
  lineVector(2,0) = cylinderCoefficients->values[5];

  Eigen::Vector3d projectedCentroid;

  projectPointToLine(linePoint, lineVector,
                     centroid.head<3>(),
                     projectedCentroid);

  linePoint = projectedCentroid;
}

double CylinderDetector::computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                       const Eigen::Vector3d& centroid,
                                       const Eigen::Vector3d& mainAxis)
{
  double height = 0;

  Eigen::Vector3d projectedPoint;
  //project all points into the main axis of the cylinder
  for (unsigned int i = 0; i < cylinderCloud->points.size(); ++i)
  {
    Eigen::Vector3d point(cylinderCloud->points[i].x,
                          cylinderCloud->points[i].y,
                          cylinderCloud->points[i].z);
    //take the largest distance to the centroid as the height of the cylinder
    projectPointToLine(centroid, mainAxis, point, projectedPoint);

    double distance = sqrt( (projectedPoint(0,0) - centroid(0,0))*(projectedPoint(0,0) - centroid(0,0)) +
                            (projectedPoint(1,0) - centroid(1,0))*(projectedPoint(1,0) - centroid(1,0)) +
                            (projectedPoint(2,0) - centroid(2,0))*(projectedPoint(2,0) - centroid(2,0)) );

    if ( 2*distance > height )
      height = 2*distance;
  }

  return height;
}

void CylinderDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if ( (cloud->width * cloud->height) == 0)
    return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCylinderCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
  bool found = pal::cylinderSegmentation<pcl::PointXYZRGB>(pclCloud,
                                                           pclCylinderCloud,
                                                           10,
                                                           0.015, 0.08,
                                                           cylinderCoefficients);

  //filter outliers in the cylinder cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredCylinderCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( pclCylinderCloud->empty() )
    pclFilteredCylinderCloud = pclCylinderCloud;
  else
    pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud, 25, 1.0,pclFilteredCylinderCloud);

  if ( found )
  {
    print(cylinderCoefficients, pclCylinderCloud->points.size());

    Eigen::Vector3d projectedCentroid, lineVector;
    getPointAndVector(pclFilteredCylinderCloud,
                      cylinderCoefficients,
                      projectedCentroid,
                      lineVector);

    double cylinderHeight = computeHeight(pclFilteredCylinderCloud,
                                          projectedCentroid,
                                          lineVector);

//    ROS_INFO_STREAM("The cylinder centroid is (" << centroid.head<3>().transpose() <<
//                    ") and projected to its axis is (" << projectedCentroid.transpose() << ")");

    Eigen::Matrix4d cylinderTransform;
    //create a frame given the cylinder parameters (point and vector)
    pal::pointAndLineTransform(lineVector,
                               projectedCentroid,
                               cylinderTransform);

    publish(pclFilteredCylinderCloud,
            cylinderCoefficients,
            cylinderTransform,
            cylinderHeight,
            cloud->header);
  }
}

void CylinderDetector::publishPose(const geometry_msgs::Pose& pose,
                                   const std_msgs::Header& header)
{
  if ( _cylinderPosePub.getNumSubscribers() > 0 )
  {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose   = pose;
    poseMsg.header = header;
    _cylinderPosePub.publish(poseMsg);
  }
}



void CylinderDetector::publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                               const Eigen::Matrix4d& transform,
                               double cylinderHeight,
                               const std_msgs::Header& header)
{
  if ( _cylinderCloudPub.getNumSubscribers() > 0 )
  {
    pcl_conversions::toPCL(header, cylinderCloud->header);
    _cylinderCloudPub.publish(cylinderCloud);
  }

  geometry_msgs::Pose pose;
  pal::convert(transform, pose);

  if ( _cylinderPosePub.getNumSubscribers() > 0 )
    publishPose(pose, header);

  if ( _cylinderMarkerPub.getNumSubscribers() > 0 )
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose = pose;
    marker.scale.x = cylinderCoefficients->values[6]*2;
    marker.scale.y = cylinderCoefficients->values[6]*2;
    marker.scale.z = cylinderHeight;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    _cylinderMarkerPub.publish(marker);
  }

  if ( _objectPub.getNumSubscribers() > 0 )
  {
    object_recognition_msgs::RecognizedObjectArray objects;
    objects.header = header;
    object_recognition_msgs::RecognizedObject object;
    object.pose.pose.pose = pose;
    object.pose.header = header;
    objects.objects.push_back(object);
    _objectPub.publish(objects);
  }
}


void CylinderDetector::start()
{
  _cloudSub = _nh.subscribe("cloud", 1, &CylinderDetector::cloudCallback, this);
  _enabled = true;
}

void CylinderDetector::stop()
{
  _cloudSub.shutdown();
  _enabled = false;
}

void CylinderDetector::run()
{
  ros::Rate loopRate(_rate);

  double halfPeriod = 0.5*1.0/_rate;

  while ( ros::ok() )
  {
    bool anySubscriber = _cylinderPosePub.getNumSubscribers() > 0 ||
                         _cylinderCloudPub.getNumSubscribers() > 0 ||
                         _cylinderMarkerPub.getNumSubscribers() > 0 ||
                         _objectPub.getNumSubscribers() > 0;


    if ( !_enabled && anySubscriber )
    {
      ROS_INFO("Enabling node because there are subscribers");
      start();
    }
    else if ( _enabled && !anySubscriber )
    {
      ROS_INFO("Disabling node because there are no subscribers");
      stop();
    }

    //check for subscriber's callbacks
    _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

    loopRate.sleep();
  }
}


}

int main(int argc, char**argv)
{
  ros::init (argc, argv, "cylinder_detector");

  ros::NodeHandle nh, pnh("~");

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  pal::CylinderDetector detector(nh, pnh);

  detector.run();

  return 0;
}
