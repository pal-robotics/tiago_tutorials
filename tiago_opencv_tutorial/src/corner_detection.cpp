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

/** \author Job van Dieten. */
 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>


class CornerDetection
{
public:
	CornerDetection(ros::NodeHandle nh_);
	~CornerDetection();
	
protected:
	void myShiTomasi_function(cv::Mat img, cv::Mat img_gray, cv::Mat myShiTomasi_dst);
	void myHarris_function(cv::Mat img, cv::Mat img_gray);
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
	
	image_transport::ImageTransport _imageTransport;
  image_transport::Subscriber image_sub;

	int myShiTomasi_qualityLevel = 50;
	int myHarris_qualityLevel = 50;
	int max_qualityLevel = 100;
	int blockSize_shi = 3;
	int blockSize_harris = 3;
	int apertureSize = 3;
	cv::Mat myHarris_copy, myShiTomasi_copy, Mc;
	double myHarris_minVal, myHarris_maxVal, myShiTomasi_minVal, myShiTomasi_maxVal;
};

const std::string shiTomasi_win = "My ShiTomasi window";
const std::string harris_win = "My Harris window";
cv::RNG rng(12345);

CornerDetection::CornerDetection(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &CornerDetection::imageCB, this, image_transport::TransportHints("compressed"));
	
	cv::namedWindow(shiTomasi_win, CV_WINDOW_FREERATIO);
  cv::createTrackbar( " Quality Level:", shiTomasi_win, &myShiTomasi_qualityLevel, max_qualityLevel);
  cv::createTrackbar( " Block Size:", shiTomasi_win, &blockSize_shi, 25);
	cv::namedWindow(harris_win, CV_WINDOW_FREERATIO);
  cv::createTrackbar( " Quality Level:", harris_win, &myHarris_qualityLevel, max_qualityLevel);
  cv::createTrackbar( " Block Size:", harris_win, &blockSize_harris, 25);
}

CornerDetection::~CornerDetection()
{
	cv::destroyAllWindows();
}

void CornerDetection::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	if(blockSize_harris == 0)
		blockSize_harris = 1;
		if(blockSize_shi == 0)
		blockSize_shi = 1;

	cv::Mat img, img_gray, myHarris_dst, myShiTomasi_dst;
	cv_bridge::CvImagePtr cvPtr;

	try
	{
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cvPtr->image.copyTo(img);
	cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

  myHarris_dst = cv::Mat::zeros( img_gray.size(), CV_32FC(6) );
  Mc = cv::Mat::zeros( img_gray.size(), CV_32FC1 );

  cv::cornerEigenValsAndVecs( img_gray, myHarris_dst, blockSize_harris, apertureSize, cv::BORDER_DEFAULT );

  for( int j = 0; j < img_gray.rows; j++ )
		for( int i = 0; i < img_gray.cols; i++ )
		 {
			float lambda_1 = myHarris_dst.at<cv::Vec6f>(j, i)[0];
			float lambda_2 = myHarris_dst.at<cv::Vec6f>(j, i)[1];
			Mc.at<float>(j,i) = lambda_1*lambda_2 - 0.04f*pow( ( lambda_1 + lambda_2 ), 2 );
		 }

  cv::minMaxLoc( Mc, &myHarris_minVal, &myHarris_maxVal, 0, 0, cv::Mat() );

  this->myHarris_function(img, img_gray);

  myShiTomasi_dst = cv::Mat::zeros( img_gray.size(), CV_32FC1 );
  cv::cornerMinEigenVal( img_gray, myShiTomasi_dst, blockSize_shi, apertureSize, cv::BORDER_DEFAULT );
  cv::minMaxLoc( myShiTomasi_dst, &myShiTomasi_minVal, &myShiTomasi_maxVal, 0, 0, cv::Mat() );

  this->myShiTomasi_function(img, img_gray, myShiTomasi_dst);

  cv::waitKey(2);
}


void CornerDetection::myHarris_function(cv::Mat img, cv::Mat img_gray)
{
	myHarris_copy = img.clone();

	if( myHarris_qualityLevel < 1 )
		myHarris_qualityLevel = 1;

	for( int j = 0; j < img_gray.rows; j++ )
		for( int i = 0; i < img_gray.cols; i++ )
			if( Mc.at<float>(j,i) > myHarris_minVal + ( myHarris_maxVal - myHarris_minVal )*myHarris_qualityLevel/max_qualityLevel )
				cv::circle( myHarris_copy, cv::Point(i,j), 4, cv::Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );

	cv::imshow( harris_win, myHarris_copy );
}

void CornerDetection::myShiTomasi_function(cv::Mat img, cv::Mat img_gray, cv::Mat myShiTomasi_dst)
{
	myShiTomasi_copy = img.clone();

	if( myShiTomasi_qualityLevel < 1 )
		myShiTomasi_qualityLevel = 1;

	for( int j = 0; j < img_gray.rows; j++ )
		for( int i = 0; i < img_gray.cols; i++ )
			if( myShiTomasi_dst.at<float>(j,i) > myShiTomasi_minVal + ( myShiTomasi_maxVal - myShiTomasi_minVal )*myShiTomasi_qualityLevel/max_qualityLevel )
				cv::circle( myShiTomasi_copy, cv::Point(i,j), 4, cv::Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );
		
	cv::imshow( shiTomasi_win, myShiTomasi_copy );
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "FindKeypoints");

	ros::NodeHandle nh;

	CornerDetection cd(nh);

	ros::spin();
}
