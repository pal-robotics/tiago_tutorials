#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointR;

class RegionSegment
{
public:
	RegionSegment(ros::NodeHandle nh);
	~RegionSegment();
	 ros::Subscriber sub;
     ros::Publisher pub;

protected:
void region_segment(pcl::PointCloud<PointT>::Ptr cloud);
void colour_segment(pcl::PointCloud<PointR>::Ptr cloud);
void min_cut_segment(pcl::PointCloud<PointT>::Ptr cloud);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void range_image(pcl::PointCloud<PointT> cloud);
void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

};


RegionSegment::RegionSegment(ros::NodeHandle nh)
{
	sub = nh.subscribe("/xtion/depth_registered/points", 1, &RegionSegment::cloud_cb, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("region", 1);

}

RegionSegment::~RegionSegment(){}

void RegionSegment::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	pcl::fromROSMsg(*input, *cloud);
// this->region_segment(cloud);
//	this->min_cut_segment(cloud);

    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(*input, cloud);

    this->range_image(cloud);

	// pcl::PointCloud<PointR>::Ptr cloudRGB(new pcl::PointCloud<PointR>);
	// pcl::fromROSMsg(*input, *cloudRGB);
	// this->colour_segment(cloudRGB);
}

void RegionSegment::range_image(pcl::PointCloud<PointT> cloud)
{
    cloud.width = (uint32_t) cloud.points.size();
    cloud.height = 1;

    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& rangeImage = *range_image_ptr;
    rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);



    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (rangeImage);
    while (!range_image_widget.wasStopped ())
     {
        range_image_widget.spinOnce ();
        pcl_sleep (0.01);
    }
    exit(EXIT_SUCCESS);
}

void RegionSegment::setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                                                    look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                                    up_vector[0], up_vector[1], up_vector[2]);
}


void RegionSegment::region_segment(pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;

	sensor_msgs::PointCloud2 cloud_out;
	pcl::PointCloud<PointR>::Ptr colored_cloud = reg.getColoredCloud ();

	pcl::toROSMsg(*colored_cloud, cloud_out);
	cloud_out.header.frame_id = "base_footprint";

	pub.publish(cloud_out);

}

void RegionSegment::min_cut_segment(pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::MinCutSegmentation<PointT> seg;
	seg.setInputCloud (cloud);
	seg.setIndices (indices);

	pcl::PointCloud<PointT>::Ptr foreground_points(new pcl::PointCloud<PointT> ());
	PointT point;
	point.x = 68.97;
	point.y = -18.55;
	point.z = 0.57;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints (foreground_points);

	seg.setSigma (0.25);
	seg.setRadius (3.0433856);
	seg.setNumberOfNeighbours (14);
	seg.setSourceWeight (0.8);

	std::vector <pcl::PointIndices> clusters;
	seg.extract (clusters);

	std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;
	
	sensor_msgs::PointCloud2 cloud_out;
	pcl::PointCloud <PointR>::Ptr colored_cloud = seg.getColoredCloud ();
	pcl::toROSMsg(*colored_cloud, cloud_out);
	cloud_out.header.frame_id = "base_footprint";

	pub.publish(cloud_out);
}

void RegionSegment::colour_segment(pcl::PointCloud<PointR>::Ptr cloud)
{
	pcl::search::Search <PointR>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointR> > (new pcl::search::KdTree<PointR>);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<PointR> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::RegionGrowingRGB<PointR> reg;
	reg.setInputCloud (cloud);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (10);
	reg.setPointColorThreshold (6);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (600);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	sensor_msgs::PointCloud2 cloud_out;
	pcl::PointCloud<PointR>::Ptr colored_cloud = reg.getColoredCloud ();

	pcl::toROSMsg(*colored_cloud, cloud_out);
	cloud_out.header.frame_id = "base_footprint";

	pub.publish(cloud_out);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "RegionSegment");
	ros::NodeHandle nh;
	RegionSegment rs(nh);
	ros::spin();
	return 0;
}
