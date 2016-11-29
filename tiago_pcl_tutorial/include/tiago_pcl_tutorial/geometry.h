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

#ifndef _PAL_PCL_GEOMETRY_H_
#define _PAL_PCL_GEOMETRY_H_

// PCL headers
#include <pcl/ModelCoefficients.h>

// Eigen headers
#include <Eigen/Core>

// Std C++ headers
#include <vector>

namespace pal {

/**
 * @brief planeTransform computes a 4x4 homogeneous transform expressing the pose of an orthonormal
 *        base of the given plane in the reference frame
 * @param planeCoeff
 * @param centroid optional point that will be the origin of the new frame. The point will be projected on the plane.
 * @param base resulting homogeneous trasnform between the reference frame and an orthonormal base of the plane
 */
void planeTransform(const pcl::ModelCoefficientsPtr& planeCoeff,
                    Eigen::Vector3d* point,
                    Eigen::Matrix4d& transform);

/**
 * @brief pointAndLineTransform given a line and a point lying on the line it computes an orthonormal base with origin
 *        set on the given point and with the Z axis parallel to the line vector.
 * @param lineVector
 * @param linePoint
 * @param transform 4x4 homogeneous transform expressing the pose of the new base in the reference frame
 */
void pointAndLineTransform(const Eigen::Vector3d& lineVector,
                           const Eigen::Vector3d& linePoint,
                           Eigen::Matrix4d& transform);


} //pal

#endif //_PAL_PCL_GEOMETRY_H_
