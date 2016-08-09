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
//    void featuresDetection(cv::Mat &img_, std::vector<cv::KeyPoint> &vector_);
//    void matchVectors(cv::Mat &img_feed, std::vector<cv::KeyPoint>& vector_feed, cv::Mat &img_aruco, std::vector<cv::KeyPoint>& vector_aruco, std::vector<cv::DMatch>& good_matches, cv::Mat& matches_matrix);
    void homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match, cv::Mat aruco_img, cv::Mat matches_mat);
    void orbMatching(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out);
    void siftMatching(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out);

	image_transport::ImageTransport _imageTransport;
    image_transport::Subscriber image_sub;
};

const std::string orb_win = "ORB";
const std::string sift_win = "SIFT";


const std::string path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/aruco.jpg";
//const std::string path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/frame0003.jpg";

FlannMatching::FlannMatching(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FlannMatching::imageCB, this, image_transport::TransportHints("compressed"));
    cv::namedWindow(orb_win, CV_WINDOW_FREERATIO);
    cv::namedWindow(sift_win, CV_WINDOW_FREERATIO);
//    cv::namedWindow(win3, CV_WINDOW_FREERATIO);
}

FlannMatching::~FlannMatching()
{
    cv::destroyAllWindows();
}

void FlannMatching::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img, orb_mat, sift_mat;

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


    cv::Mat img_aruco = cv::imread(path);
    if(!img_aruco.data)
        ROS_INFO("NO DATA");

    this->orbMatching(img, img_aruco, orb_mat);
    this->siftMatching(img, img_aruco, sift_mat);

    cv::imshow(sift_win, sift_mat);
    cv::imshow(orb_win, orb_mat);

    cv::waitKey(1);
}

void FlannMatching::orbMatching(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out)
{
    cv::Mat desc_feed, desc_static;
    cv::OrbFeatureDetector detector;

    std::vector<cv::KeyPoint> vec_feed, vec_static;

    detector.detect(in_feed, vec_feed);
    detector.detect(in_static, vec_static);

    cv::OrbDescriptorExtractor extractor;

    extractor.compute(in_feed, vec_feed, desc_feed);
    extractor.compute(in_static, vec_static, desc_static);

    cv::BFMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(desc_feed, desc_static, matches, 2);

    std::vector<cv::DMatch> good_matches;

    for(const auto m : matches)
        if(m[0].distance<m[1].distance*0.6)
            good_matches.push_back(m[0]);

    cv::drawMatches(in_feed, vec_feed, in_static, vec_static, good_matches, out);
}

void FlannMatching::siftMatching(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out)
{
    cv::Mat desc_feed, desc_static;
    cv::SiftFeatureDetector detector;

    std::vector<cv::KeyPoint> vec_feed, vec_static;

    detector.detect(in_feed, vec_feed);
    detector.detect(in_static, vec_static);

    cv::SiftDescriptorExtractor extractor;

    extractor.compute(in_feed, vec_feed, desc_feed);
    extractor.compute(in_static, vec_static, desc_static);

    cv::BFMatcher matcher;
    std::vector<std::vector<cv::DMatch>>matches;
    matcher.knnMatch(desc_feed, desc_static, matches, 2);

    std::vector<cv::DMatch> good_matches;

    for(const auto m : matches)
        if(m[0].distance<m[1].distance*0.6)
            good_matches.push_back(m[0]);

    cv::drawMatches(in_feed, vec_feed, in_static, vec_static, good_matches, out);
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

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FlannMatching");

  ros::NodeHandle nh;

  FlannMatching fm(nh);

  ros::spin();
}
