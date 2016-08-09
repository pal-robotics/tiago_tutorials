// ROS HEADERS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <opencv_tut/valueMatrix.h>

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
    void matrixCB(const opencv_tut::valueMatrixConstPtr& msg);
    void surfFeaturesDetection(cv::Mat img_, cv::Mat& out_);
    void contrastChange(cv::Mat tochange);
    void sharpenImg(cv::Mat img);
	image_transport::ImageTransport _imageTransport;
    image_transport::Subscriber image_sub;
    ros::Subscriber matrix_sub;
    int alpha, beta;
    int zero_zero, zero_one, zero_two, one_zero, one_one, one_two, two_zero, two_one, two_two;
    bool keypoints_bool, sharpen_bool, contrast_bool, original_bool, combined_bool;
};

const std::string win1 = "KeyPoints";
const std::string win2 = "Sharper";
const std::string win3 = "Contrast";

const std::string key_win= "Keypoints";
const std::string shar_win= "Sharpen Image";
const std::string cont_win = "Contrast Manipulation";
const std::string orig_win = "Original Image";
const std::string combi_win= "Combined Effects";


const std::string img_path = "/home/job/pal_robotics_catkin/src/tiago_tutorials/opencv_tut/resources/aruco.jpg";

FindKeypoints::FindKeypoints(ros::NodeHandle nh_): _imageTransport(nh_)
{
    alpha = 2.2;
    beta = 50;
    zero_zero = 0; zero_one = -1; zero_two = 0;
    one_zero = -1; one_one = 5; one_two = -1;
    two_zero = 0; two_one = -1; two_two = 0;

    cv::namedWindow(win1, CV_WINDOW_FREERATIO);
    cv::namedWindow(win2, CV_WINDOW_FREERATIO);
    cv::namedWindow(win3, CV_WINDOW_FREERATIO);
    image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &FindKeypoints::imageCB, this, image_transport::TransportHints("compressed"));
    matrix_sub = nh_.subscribe("/opencv_tut/Matrix_values", 1, &FindKeypoints::matrixCB, this);
}

FindKeypoints::~FindKeypoints()
{
    cv::destroyAllWindows();
}

void FindKeypoints::matrixCB(const opencv_tut::valueMatrixConstPtr& msg)
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
    cv::Mat img_aruco = cv::imread(img_path, 1);
//    this->surfFeaturesDetection(img_aruco, img_aruco);

    cvPtr->image.copyTo(img);
    this->contrastChange(img);

    this->surfFeaturesDetection(img, out);
    this->sharpenImg(img);
    cv::imshow(win1, out);
    cv::waitKey(2);
}

void FindKeypoints::sharpenImg(cv::Mat img)
{

    cv::Mat sharper = cv::Mat::zeros(img.size(), img.type());

    cv::Mat kern = (cv::Mat_<char>(3,3) << zero_zero, zero_one, zero_two,
                                                                     one_zero, one_one, one_two,
                                                                    zero_two, one_one, one_two);

    cv::filter2D(img, sharper, img.depth(), kern);
    cv::imshow(win2, sharper);

}

void FindKeypoints::contrastChange(cv::Mat tochange)
{
    cv::Mat processed = cv::Mat::zeros(tochange.size(), tochange.type());
    for(int i = 0; i<tochange.rows; i++)
        for(int j = 0; j<tochange.cols; j++)
            for(int k = 0; k<3; ++k)
                processed.at<cv::Vec3b>(i,j).val[k] = cv::saturate_cast<uchar>(alpha * (tochange.at<cv::Vec3b>(i,j).val[k] + beta));

    cv::imshow(win3, processed);
}

void FindKeypoints::surfFeaturesDetection(cv::Mat img_, cv::Mat& out_)
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
