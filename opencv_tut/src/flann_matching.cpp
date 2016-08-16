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

//Custom Headers
#include <opencv_tut/valueMatrix.h>


class FlannMatching
{
public:
	FlannMatching(ros::NodeHandle nh_);
	~FlannMatching();
	
protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
    void guiCB(const opencv_tut::valueMatrixConstPtr& msg);
    void homography(std::vector<cv::KeyPoint> aruco_, std::vector<cv::KeyPoint> feed_, std::vector<cv::DMatch> match, cv::Mat aruco_img, cv::Mat matches_mat);
    void Matcher(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out);


	image_transport::ImageTransport _imageTransport;
    image_transport::Subscriber image_sub;
    ros::Subscriber  gui_sub;


    std::string extractor_gui, feature_gui, matcher_gui;
    bool knn;
    int k;
    float dist_check;
    std::string path;
};

//const std::string orb_win = "ORB";
const std::string win = "Matcher";


FlannMatching::FlannMatching(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FlannMatching::imageCB, this, image_transport::TransportHints("compressed"));
    gui_sub = nh_.subscribe("/opencv_tut/flann_matching_gui", 1, &FlannMatching::guiCB, this);
    cv::namedWindow(win, CV_WINDOW_FREERATIO);
    feature_gui = "SURF";
    extractor_gui = "SURF";
    matcher_gui = "FlannBased";
    knn = false;
    k = 2;
    dist_check = 0.6;
    path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/aruco.jpg";
}

FlannMatching::~FlannMatching()
{
    cv::destroyAllWindows();
}

void FlannMatching::guiCB(const opencv_tut::valueMatrixConstPtr& msg)
{
    if(msg->header.frame_id == "dist")
        dist_check = msg->value;
    else if(msg->header.frame_id == "k")
        k = msg->value;
    else if(msg->header.frame_id == "k")
        k = msg->value;
    else if(msg->header.frame_id == "dist_check")
        dist_check = msg->value;
    else if(msg->header.frame_id == "path")
        path = msg->option;
    else if(msg->header.frame_id == "Keypoints")
        knn = msg->tick;
    else if(msg->header.frame_id == "feature_choice")
        feature_gui = msg->option;
    else if(msg->header.frame_id == "extracter_choice")
        extractor_gui = msg->option;
    else if(msg->header.frame_id == "matcher_choice")
        matcher_gui = msg->option;
}
void FlannMatching::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img, orb_mat, sift_mat, new_mat;

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

    cv::Mat img_stat = cv::imread(path);
    if(!img_stat.data)
        ROS_INFO("NO DATA");
    if(!img.data)
        ROS_INFO("NOD ATA");

    this->Matcher(img, img_stat, new_mat);
    cv::imshow(win, new_mat);
    cv::waitKey(1);
}

void FlannMatching::Matcher(cv::Mat in_feed,cv::Mat in_static ,cv::Mat& out)
{
    cv::initModule_nonfree();

    cv::Mat desc_feed, desc_static;
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(feature_gui);

    std::vector<cv::KeyPoint> vec_feed, vec_static;

    detector->detect(in_feed, vec_feed);
    detector->detect(in_static, vec_static);

    cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(extractor_gui);

    extractor->compute(in_feed, vec_feed, desc_feed);
    extractor->compute(in_static, vec_static, desc_static);

    if(desc_feed.type() != CV_32F || desc_static.type() != CV_32F)
    {
        desc_feed.convertTo(desc_feed, CV_32F);
        desc_static.convertTo(desc_static, CV_32F);
    }
    if(matcher_gui == "BruteForce-Hamming" || matcher_gui == "BruteForce-Hamming(2)")
    {
        desc_feed.convertTo(desc_feed, CV_8U);
        desc_static.convertTo(desc_static, CV_8U);
    }

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matcher_gui);


    std::vector<std::vector<cv::DMatch>> matches;
////    if(knn == true)
        matcher->knnMatch(desc_feed, desc_static, matches, k);

    std::vector<cv::DMatch> good_matches;

    for(const auto m : matches)
        if(m[0].distance<m[1].distance*dist_check)
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
