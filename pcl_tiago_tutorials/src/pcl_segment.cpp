#include "pcl_segment.h"

pcl_segment::pcl_segment(ros::NodeHandle nh_)
{
	sub = nh_.subscribe("/xtion/depth_registered/points", 1, &pcl_segment::cloud_cb, this);
	pub = nh_.advertise<pcl::PCLPointCloud2>("filtered", 1);
	pub_1 = nh_.advertise<pcl::PCLPointCloud2>("extracted_surface", 1);
	pub_2 = nh_.advertise<pcl::PCLPointCloud2>("original_without_extracted_surface", 1);
	f = boost::bind(&pcl_segment::dynam_CB, this, _1, _2);
	server.setCallback(f);

}

pcl_segment::~pcl_segment(){}

void pcl_segment::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud);
	pcl_segment::filter(cloud);
}

void pcl_segment::dynam_CB(pcl_tiago::pclConfig& config, uint32_t level)
{

	setLeafX = config.setLeafSize_X;
	setLeafY = config.setLeafSize_Y;
	setLeafZ = config.setLeafSize_Z;
	MaxIt = config.Max_Iterations;
	DistThresh = config.Distance_Threshold;
	optimize = config.Optimize_Coefficients;
	choose_image = config.image;

	ROS_INFO_STREAM("Leaf Size (" << setLeafX << ", "<< setLeafY << ", " << setLeafZ << ")");
	ROS_INFO_STREAM("Max Iterations: " << MaxIt << " - Distance Threshold: " << DistThresh);
}

void pcl_segment::filter(pcl::PCLPointCloud2::Ptr cloud_)
{	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
	sor.setInputCloud(cloud_);		
	sor.setLeafSize (setLeafX, setLeafY, setLeafZ);
	sor.filter(*cloud_filtered);
	pub.publish(cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_points (new pcl::PointCloud<pcl::PointXYZ>); 
	
	if(choose_image == 0)
		pcl::fromPCLPointCloud2(*cloud_, *cloud_filtered_points);
	else if(choose_image == 1)
		pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filtered_points);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (optimize);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations(MaxIt);
	seg.setDistanceThreshold (DistThresh);
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
  ros::init(argc, argv, "pcl_segment");
  ros::NodeHandle nh;
  pcl_segment pcl_object(nh);
  ros::spin();
  return 0;
}