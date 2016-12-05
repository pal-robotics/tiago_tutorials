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
#include <tiago_pcl_tutorial/geometry.h>

namespace pal {

void normalizeVector(std::vector<double>& v)
{
  double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  v[0] /= norm;
  v[1] /= norm;
  v[2] /= norm;
}

/**
 * @brief planeOrthonormalBase compute a orthonormal base of the plane expressed in the reference frame so that
 *        the Z axis of the new base corresponds to the normal vector to the plane
 * @param planeCoeff
 * @param v orientations of the X,Y and Z vectors of the new base expressed in the reference one
 */
void planeOrthonormalBase(const pcl::ModelCoefficientsPtr& planeCoeff,
                          std::vector<double> (&v)[3])
{
  double A = planeCoeff->values[0];
  double B = planeCoeff->values[1];
  double C = planeCoeff->values[2];
  double D = planeCoeff->values[3];

  //first vector of the base: plane normal Z = (A, B, C)
  v[2].push_back(A);
  v[2].push_back(B);
  v[2].push_back(C);
  normalizeVector(v[2]);

  A = v[2][0];
  B = v[2][1];
  C = v[2][2];

  //second vector:
  //if |C| > |A| and |C| > |B|:
  //  vector: ( X=1, Y=0, Z=-(D+A)/C )
  //if |A| > |B| and |A| > |C|:
  //  vector: ( X=-(B+D)/A, Y=1, Z=0 )
  //if |B| > |A| and |B| > |C|:
  //  vector: ( X=1, Y=-(A+D)/B, Z=0 )
  if ( fabs(C) >= fabs(A) && fabs(C) >= fabs(B) )
  {
    v[0].push_back(1);
    v[0].push_back(0);
    v[0].push_back(-A/C);
  }
  else if ( fabs(A) >= fabs(B) && fabs(A) >= fabs(C) )
  {
    v[0].push_back(-B/A);
    v[0].push_back(1);
    v[0].push_back(0);
  }
  else if ( fabs(B) >= fabs(A) && fabs(B) >= fabs(C) )
  {
    v[0].push_back(1);
    v[0].push_back(-A/B);
    v[0].push_back(0);
  }
  normalizeVector(v[0]);

  //third vector: cross vector of the first two
  v[1].push_back(v[2][2]*v[0][1] - v[2][1]*v[0][2]);  // n_z·v_y - n_y·v_z
  v[1].push_back(v[2][0]*v[0][2] - v[2][2]*v[0][0]);  // n_x·v_z - n_z·v_x
  v[1].push_back(v[2][1]*v[0][0] - v[2][0]*v[0][1]);  // n_y·v_x - n_x·v_y
  normalizeVector(v[1]);

  //Compute a point of the plane
}

void planeTransform(const pcl::ModelCoefficientsPtr& planeCoeff,
                    Eigen::Vector3d* point,
                    Eigen::Matrix4d& transform)
{
  std::vector<double> v[3];

  pal::planeOrthonormalBase(planeCoeff, v);

  //now compute a point belonging to the plane
  double A = planeCoeff->values[0];
  double B = planeCoeff->values[1];
  double C = planeCoeff->values[2];
  double D = planeCoeff->values[3];
  double X, Y, Z;

  //choose the plane origin as follows:
  if ( fabs(C) >= fabs(A) && fabs(C) >= fabs(B) )
  {
    X = 0; Y = 0; Z = -D/C;
  }
  else if ( fabs(A) >= fabs(B) && fabs(A) >= fabs(C) )
  {
    X = -D/A; Y = 0; Z = 0;
  }
  else if ( fabs(B) >= fabs(A) && fabs(B) >= fabs(C) )
  {
    X = 0; Y = -D/B; Z = 0;
  }
  else
  {
      // otherwise these variables are used uninitialized below
      // throwing an exception is probably better, even if it never happens
      X = 0;
      Y = 0;
      Z = 0;
       //std::runtime_exception("Invalid plane coefficients");
  }

  //if a point is provided
  if ( point != NULL )
  {
    //project the point to the plane
    double t = A*X-A*(*point)(0,0) + B*Y-B*(*point)(1,0) + C*Z-C*(*point)(2,0);
    X = (*point)(0,0) + t*A;
    Y = (*point)(1,0) + t*B;
    Z = (*point)(2,0) + t*C;   
  }

  //fill in the transform as a Eigen::Matrix4d
  transform(0,0) = v[0][0]; transform(0,1) = v[1][0]; transform(0,2) = v[2][0]; transform(0,3) = X;
  transform(1,0) = v[0][1]; transform(1,1) = v[1][1]; transform(1,2) = v[2][1]; transform(1,3) = Y;
  transform(2,0) = v[0][2]; transform(2,1) = v[1][2]; transform(2,2) = v[2][2]; transform(2,3) = Z;
  transform(3,0) =       0; transform(3,1) =       0; transform(3,2) =       0; transform(3,3) = 1;
}


void pointAndLineTransform(const Eigen::Vector3d& lineVector,
                           const Eigen::Vector3d& linePoint,
                           Eigen::Matrix4d& transform)
{
  //define a plane containing the line point and with normal vector equal to the line vector
  pcl::ModelCoefficientsPtr planeCoefficients( new pcl::ModelCoefficients );

  planeCoefficients->values.resize(4);

  Eigen::Vector3d normal;
  normal = lineVector.normalized();
  planeCoefficients->values[0] = normal(0,0);
  planeCoefficients->values[1] = normal(1,0);
  planeCoefficients->values[2] = normal(2,0);
  // Provided the plane explicit equation A·X + B·Y + C·Z + D = 0, define D by
  // evaluating the equation with the line point:
  planeCoefficients->values[3] = -( normal(0,0)*linePoint(0,0) +
                                    normal(1,0)*linePoint(1,0) +
                                    normal(2,0)*linePoint(2,0) );

  Eigen::Vector3d origin;
  origin = linePoint;

  pal::planeTransform(planeCoefficients,
                      &origin,
                      transform);
}


} //pal
