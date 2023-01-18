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

// ROS HEADERS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <tiago_opencv_tutorial/valueMatrix.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifndef NO_CV_XFEATURES2D
#include <opencv2/xfeatures2d/nonfree.hpp>
#endif
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "features2d_factory.h"
class FindKeypoints
{
public:
	FindKeypoints(ros::NodeHandle nh_);
	~FindKeypoints();
protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
	void matrixCB(const tiago_opencv_tutorial::valueMatrixConstPtr& msg);
	void FeaturesDetection(cv::Mat in, cv::Mat& out);
	void contrastChange(cv::Mat in, cv::Mat& out);
	void sharpenImg(cv::Mat in, cv::Mat& out);
	void cvWindows();

	image_transport::ImageTransport _imageTransport;
	image_transport::Subscriber image_sub;
	ros::Subscriber gui_sub;

	float alpha;
	int beta;
	int zero_zero, zero_one, zero_two, one_zero, one_one, one_two, two_zero, two_one, two_two;
	bool keypoints_bool, sharpen_bool, contrast_bool, original_bool, combined_bool;
	bool keypoints_show, sharpen_show, contrast_show, original_show, combined_show;
	std::string detector_from_msg;
};

const std::string key_win= "Keypoints";
const std::string sharp_win= "Sharpen Image";
const std::string cont_win = "Contrast Manipulation";
const std::string orig_win = "Original Image";
const std::string combi_win= "Combined Effects";

FindKeypoints::FindKeypoints(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FindKeypoints::imageCB, this, image_transport::TransportHints("compressed"));
	gui_sub = nh_.subscribe("/tiago_opencv_tutorial/find_keypoints_gui", 1, &FindKeypoints::matrixCB, this);
	alpha = 2.2;
	beta = 50;
	zero_zero = 0; zero_one = -1; zero_two = 0;
	one_zero = -1; one_one = 5; one_two = -1;
	two_zero = 0; two_one = -1; two_two = 0;
	keypoints_bool= true;
	sharpen_bool = contrast_bool = original_bool = combined_bool = false;
	keypoints_show = sharpen_show = contrast_show = original_show = combined_show = false;
	detector_from_msg = "ORB";
}

FindKeypoints::~FindKeypoints()
{
	cv::destroyAllWindows();
}

void FindKeypoints::matrixCB(const tiago_opencv_tutorial::valueMatrixConstPtr& msg)
{
	if(msg->header.frame_id == "zero_zero")
		zero_zero = msg->value;
	else if(msg->header.frame_id == "zero_one")
		zero_one = msg->value;
	else if(msg->header.frame_id == "zero_two")
		zero_two = msg->value;
	else if(msg->header.frame_id == "one_zero")
		one_zero = msg->value;
	else if(msg->header.frame_id == "one_one")
		one_one = msg->value;
	else if(msg->header.frame_id == "one_two")
		one_two = msg->value;
	else if(msg->header.frame_id == "two_zero")
		two_zero = msg->value;
	else if(msg->header.frame_id == "two_one")
		two_one = msg->value;
	else if(msg->header.frame_id == "two_two")
		two_two = msg->value;
	else if(msg->header.frame_id == "alpha")
		alpha = msg->value;
	else if(msg->header.frame_id == "beta")
		beta = msg->value;
	else if(msg->header.frame_id == "Keypoints")
		keypoints_bool = msg->tick;
	else if(msg->header.frame_id == "Sharpen")
		sharpen_bool = msg->tick;
	else if(msg->header.frame_id == "Contrast")
		contrast_bool = msg->tick;
	else if(msg->header.frame_id == "Original")
		original_bool = msg->tick;
	else if(msg->header.frame_id == "Combined")
		combined_bool = msg->tick;
	else
		detector_from_msg = msg->header.frame_id;

	this->cvWindows();
}

void FindKeypoints::cvWindows()
{
	if(keypoints_bool == true)
		cv::namedWindow(key_win, cv::WINDOW_FREERATIO);
	else if (keypoints_bool == false && keypoints_show == true)
	{
		cv::destroyWindow(key_win);
		keypoints_show = false;
	}
	if(sharpen_bool == true)
		cv::namedWindow(sharp_win, cv::WINDOW_FREERATIO);
	else if (sharpen_bool == false && sharpen_show == true)
	{
		cv::destroyWindow(sharp_win);
		sharpen_show = false;
	}
	if(contrast_bool == true)
		cv::namedWindow(cont_win, cv::WINDOW_FREERATIO);
	else if (contrast_bool == false && contrast_show == true)
	{
		cv::destroyWindow(cont_win);
		contrast_show = false;
	}
	if(original_bool == true)
		cv::namedWindow(orig_win, cv::WINDOW_FREERATIO);
	else if (original_bool == false && original_show == true)
	{
		cv::destroyWindow(orig_win);
		original_show = false;
	}
	if(combined_bool == true)
		cv::namedWindow(combi_win, cv::WINDOW_FREERATIO);
	else if (combined_bool == false && combined_show == true)
	{
		cv::destroyWindow(combi_win);
		combined_show = false;
	}
}

void FindKeypoints::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat img, keypoints_mat, sharp_mat, contrast_mat, combi_mat;
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

	if(original_bool == true)
	{
		original_show = true;
		cv::imshow(orig_win, img);
	}
	if(contrast_bool == true)
	{
		contrast_show = true;
		this->contrastChange(img, contrast_mat);
		cv::imshow(cont_win, contrast_mat);
	}
	if(sharpen_bool == true)
	{
		sharpen_show = true;
		this->sharpenImg(img, sharp_mat);
		cv::imshow(sharp_win, sharp_mat);
	}
	if(keypoints_bool == true)
	{
		keypoints_show = true;
		this->FeaturesDetection(img, keypoints_mat);
		cv::imshow(key_win, keypoints_mat);
	}
	if(combined_bool == true)
	{
		combined_show = true;
		cv::Mat combi_mat_1, combi_mat_2;
		this->contrastChange(img, combi_mat_1);
		this->sharpenImg(combi_mat_1, combi_mat_2);
		this->FeaturesDetection(combi_mat_2, combi_mat);
		cv::imshow(combi_win, combi_mat);
	}
	cv::waitKey(2);
}

void FindKeypoints::sharpenImg(cv::Mat in, cv::Mat& out)
{
	out = cv::Mat::zeros(in.size(), in.type());
	cv::Mat kern = (cv::Mat_<char>(3,3) << zero_zero, zero_one, zero_two,
																					one_zero, one_one, one_two,
																					zero_two, one_one, one_two);
	cv::filter2D(in, out, in.depth(), kern);
}

void FindKeypoints::contrastChange(cv::Mat in, cv::Mat& out)
{
	out = cv::Mat::zeros(in.size(), in.type());
	for(int i = 0; i<in.rows; i++)
		for(int j = 0; j<in.cols; j++)
			for(int k = 0; k<3; ++k)
				out.at<cv::Vec3b>(i,j).val[k] = cv::saturate_cast<uchar>(alpha * (in.at<cv::Vec3b>(i,j).val[k] + beta));
}

void FindKeypoints::FeaturesDetection(cv::Mat in, cv::Mat& out)
{
	std::vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::FeatureDetector> detector = create(detector_from_msg);
	detector->detect(in, keypoints);
	cv::drawKeypoints(in, keypoints, out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FindKeypoints");

  ros::NodeHandle nh;

  FindKeypoints fk(nh);

  ros::spin();
}
