#include "pcl_test.h"

pcl_test::pcl_test(ros::NodeHandle nh_)
{
	sub = nh_.subscribe("/xtion/depth_registered/points", 1, &pcl_test::cloud_cb, this);
	pub = nh_.advertise<pcl::PCLPointCloud2>("filtered", 1);
	pub_1 = nh_.advertise<pcl::PCLPointCloud2>("extracted_surface", 1);
	pub_2 = nh_.advertise<pcl::PCLPointCloud2>("original_without_extracted_surface", 1);
	f = boost::bind(&pcl_test::dynam_CB, this, _1, _2);
	server.setCallback(f);

}

pcl_test::~pcl_test(){}

void pcl_test::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud);
	pcl_test::filter(cloud);
}

void pcl_test::dynam_CB(pcl_tiago::pclConfig& config, uint32_t level)
{
	// ROS_INFO("Reconfigure Request: %d %f %s %s %d",
	//         config.int_param, config.double_param, 
 //            config.str_param.c_str(), 
 //            config.bool_param?"True":"False", 
 //            config.size);
	setLeafX = config.setLeafSize_X;
	setLeafY = config.setLeafSize_Y;
	setLeafZ = config.setLeafSize_Z;
	MaxIt = config.Max_Iterations;
	DistThresh = config.Distance_Threshold;

	ROS_INFO_STREAM("Leaf Size (" << setLeafX << ", "<< setLeafY << ", " << setLeafZ << ")");
	ROS_INFO_STREAM("Max Iterations: " << MaxIt << " - Distance Threshold: " << DistThresh);
}

void pcl_test::filter(pcl::PCLPointCloud2::Ptr cloud_)
{	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
	sor.setInputCloud(cloud_);		
	sor.setLeafSize (setLeafX, setLeafY, setLeafZ/*0.02f, 0.02f, 0.02f*/);
	sor.filter(*cloud_filtered);
	pub.publish(cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_points (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::fromPCLPointCloud2(*cloud_, *cloud_filtered_points);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud_filtered_points);
	seg.segment (*inliers, *coefficients);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>); 

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud_filtered_points);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_1);
	extract.setNegative(true);
	extract.filter(*cloud_2);

	pcl::toPCLPointCloud2(*cloud_1, *cloud_);
	pub_1.publish(cloud_);

	pcl::toPCLPointCloud2(*cloud_2, *cloud_);
	pub_2.publish(cloud_);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_test");
  ros::NodeHandle nh;
  pcl_test pcl_object(nh);

  // dynamic_reconfigure::Server<pcl_tiago::pclConfig> server;
  // dynamic_reconfigure::Server<pcl_tiago::pclConfig> f;
  // f = boost::bind(&/*pcl_object->*/dynam_CB, _1, _2);
  // server.setCallback(f);

  ros::spin();
  return 0;
}