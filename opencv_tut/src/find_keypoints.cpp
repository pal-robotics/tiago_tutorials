// ROS HEADERS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>


class FindKeypoints
{
public:
	FindKeypoints(ros::NodeHandle nh_);
	~FindKeypoints();
	
protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
	void featuresDetection(cv::Mat img_, cv::Mat& out_);
	image_transport::ImageTransport _imageTransport;
  image_transport::Subscriber image_sub;

  std::string win1, win2;
};


FindKeypoints::FindKeypoints(ros::NodeHandle nh_): _imageTransport(nh_)
{
	win1 = "Live Feed";
	win2 = "Keypoints";
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FindKeypoints::imageCB, this, image_transport::TransportHints("compressed"));
	cv::namedWindow(win1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win2, CV_WINDOW_AUTOSIZE);
}

FindKeypoints::~FindKeypoints()
{
	cv::destroyWindow(win1);
	cv::destroyWindow(win2);
}

void FindKeypoints::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat img, out;
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
  this->featuresDetection(img, out);
  cv::imshow(win1, img);
  cv::imshow(win2, out);
  cv::waitKey(0);
}

void FindKeypoints::featuresDetection(cv::Mat img_, cv::Mat& out_)
{
	cv::SurfFeatureDetector detector(400);

	std::vector<cv::KeyPoint> keypoints;

	detector.detect(img_, keypoints);

	cv::drawKeypoints(img_, keypoints, out_, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FindKeypoints");

  ros::NodeHandle nh;

  FindKeypoints fk(nh);

  ros::spin();
}