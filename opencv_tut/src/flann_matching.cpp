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
#include <opencv2/calib3d/calib3d.hpp>


class FlannMatching
{
public:
	FlannMatching(ros::NodeHandle nh_);
	~FlannMatching();
	
protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
    void featuresDetection(cv::Mat &img_, std::vector<cv::KeyPoint> &vector_);
    void matchVectors(cv::Mat &img_feed, std::vector<cv::KeyPoint>& vector_feed, cv::Mat &img_aruco, std::vector<cv::KeyPoint>& vector_aruco, std::vector<cv::DMatch>& good_matches, cv::Mat& matches_matrix);
    void homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match, cv::Mat aruco_img, cv::Mat matches_mat);

	image_transport::ImageTransport _imageTransport;
    image_transport::Subscriber image_sub;
};

const std::string win1 = "Feed";
const std::string win2 = "Aruco";
const std::string win3 = "Matches";
const std::string path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/aruco.jpg";
//const std::string path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/frame0003.jpg";

FlannMatching::FlannMatching(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FlannMatching::imageCB, this, image_transport::TransportHints("compressed"));
//    cv::namedWindow(win1, CV_WINDOW_FREERATIO);
//    cv::namedWindow(win2, CV_WINDOW_FREERATIO);
    cv::namedWindow(win3, CV_WINDOW_FREERATIO);
}

FlannMatching::~FlannMatching()
{
    cv::destroyAllWindows();
}

void FlannMatching::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat img, out;
	cv_bridge::CvImagePtr cvPtr;
    std::vector<cv::DMatch> good_matches;
    cv::Mat matches_matrix;

	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
    cv::Mat img_aruco = cv::imread(path);
    if(!img_aruco.data)
        ROS_INFO("NO DATA");
    std::vector<cv::KeyPoint> key_aruco;
    this->featuresDetection(img_aruco, key_aruco);

    cvPtr->image.copyTo(img);
    std::vector<cv::KeyPoint> key_img;
    this->featuresDetection(img, key_img);
    this->matchVectors(img, key_img, img_aruco, key_aruco, good_matches, matches_matrix);
//    this->homography(key_aruco, key_img, good_matches, img_aruco, matches_matrix);

//    cv::imshow(win1, img_aruco);
//    cv::imshow(win2, img);
    cv::waitKey(4);
}

void FlannMatching::featuresDetection(cv::Mat& img_, std::vector<cv::KeyPoint>& vector_)
{
    cv::SiftFeatureDetector detector(400);

    detector.detect(img_, vector_);

    cv::drawKeypoints(img_, vector_, img_, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
}

void FlannMatching::matchVectors(cv::Mat& img_feed, std::vector<cv::KeyPoint>& vector_feed, cv::Mat& img_aruco_, std::vector<cv::KeyPoint>& vector_aruco, std::vector<cv::DMatch>& good_matches, cv::Mat& matches_matrix)
{
    cv::Mat desc_feed, desc_aruco;

    cv::SiftDescriptorExtractor extract;

    extract.compute(img_feed, vector_feed, desc_feed);
    extract.compute(img_aruco_, vector_aruco, desc_aruco);

    cv::BFMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(desc_feed, desc_aruco, matches,2);

    for(const auto d : matches)
    {
        if(d[0].distance<d[1].distance*0.6)
            good_matches.push_back(d[0]);
    }

//    ROS_INFO_STREAM("good matches : " << good_matches.size());

    cv::drawMatches(img_feed, vector_feed, img_aruco_, vector_aruco, good_matches, matches_matrix);
//    cv::drawMatches(img_aruco_, vector_aruco, img_feed, vector_feed, good_matches, matches_matrix);
    cv::imshow(win3, matches_matrix);
}

void FlannMatching::homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match_vector, cv::Mat aruco_img, cv::Mat matches_mat)
{
    std::vector<cv::Point2f> aruco_2f, feed_2f;
ROS_INFO("Started Homography");
    for(int i = 0; i<match_vector.size(); ++i)
    {
        aruco_2f.push_back(aruco_[match_vector[i].queryIdx].pt);
        feed_2f.push_back(feed_[match_vector[i].trainIdx].pt);
    }

    cv::Mat h_mat = cv::findHomography(aruco_2f, feed_2f, CV_RANSAC );

    std::vector<cv::Point2f>aruco_corners(4), feed_corners(4);
    aruco_corners[0] = cv::Point2f(0,0);
    aruco_corners[1] = cv::Point2f(matches_mat.cols, 0);
    aruco_corners[2] = cv::Point2f(matches_mat.cols, matches_mat.rows);
    aruco_corners[3] = cv::Point2f(0, matches_mat.rows);

    cv::perspectiveTransform(aruco_corners, feed_corners, h_mat);

    cv::line(matches_mat, feed_corners[0] + cv::Point2f(aruco_img.cols, 0), feed_corners[1] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
    cv::line(matches_mat, feed_corners[1] + cv::Point2f(aruco_img.cols, 0), feed_corners[2] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
    cv::line(matches_mat, feed_corners[2] + cv::Point2f(aruco_img.cols, 0), feed_corners[3] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);
    cv::line(matches_mat, feed_corners[3] + cv::Point2f(aruco_img.cols, 0), feed_corners[0] + cv::Point2f(aruco_img.rows), cv::Scalar(0,255,0), 4);

    cv::imshow(win2, matches_mat);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FlannMatching");

  ros::NodeHandle nh;

  FlannMatching fm(nh);

  ros::spin();
}
