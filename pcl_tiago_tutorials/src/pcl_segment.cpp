#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tiago_tutorials/pclConfig.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

class pcl_segment
{
public:
    pcl_segment(ros::NodeHandle nh_);
    ~pcl_segment();
    void segmentPlane(pcl::PCLPointCloud2::Ptr cloud_);
    void segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void segmentEuclidean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    void dynam_CB(const pcl_tiago_tutorials::pclConfig &config, uint32_t level);
    double setLeafX, setLeafY, setLeafZ, DistThresh;
    bool optimize;
    int choose_image;
    int iteration;
    pcl::PCLPointCloud2::Ptr cloud_filtered;
    ros::Subscriber sub;
    ros::Publisher pub, pub_1, pub_2, pub_3, pub_4;
    dynamic_reconfigure::Server<pcl_tiago_tutorials::pclConfig> server;
    dynamic_reconfigure::Server<pcl_tiago_tutorials::pclConfig>::CallbackType f;
};

typedef pcl::PointXYZ PointT;

pcl_segment::pcl_segment(ros::NodeHandle nh_)
{
    ROS_INFO("Started pcl_Segment");
    sub = nh_.subscribe("/xtion/depth_registered/points", 1, &pcl_segment::cloud_cb, this);
    ROS_INFO("subscribed");
	pub = nh_.advertise<pcl::PCLPointCloud2>("filtered", 1);
    ROS_INFO("published filtered");
	pub_1 = nh_.advertise<pcl::PCLPointCloud2>("extracted_surface", 1);
    ROS_INFO("published extracted_surface");
	pub_2 = nh_.advertise<pcl::PCLPointCloud2>("original_without_extracted_surface", 1);
    ROS_INFO("published original");
    pub_3 = nh_.advertise<pcl::PointCloud<PointT> >("pub_three", 1);
    ROS_INFO("published filter pclxyz");
    pub_4 = nh_.advertise<pcl::PointCloud<PointT> >("pub_four", 1);
//    pub_3 = nh_.advertise<sensor_msgs::PointCloud2>("2nd filter");
    ROS_INFO("published 2nd filter");
    f = boost::bind(&pcl_segment::dynam_CB, this, _1, _2);
	server.setCallback(f);
    iteration = 0;
    ROS_INFO("Finished creating subscribers and publishers");
}

pcl_segment::~pcl_segment(){}

void pcl_segment::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<PointT>::Ptr cloud_1(new pcl::PointCloud<PointT>);
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromROSMsg(*input, *cloud_1);
   pcl_segment::segmentPlane(cloud);
//    pcl_segment::segmentCylinder(cloud_1);
    // pcl_segment::segmentEuclidean(cloud_1);
    iteration++;
    ROS_INFO_STREAM("Iteration: " << iteration);
}

void pcl_segment::dynam_CB(const pcl_tiago_tutorials::pclConfig &config, uint32_t level)
{
	setLeafX = config.setLeafSize_X;
	setLeafY = config.setLeafSize_Y;
	setLeafZ = config.setLeafSize_Z;
	DistThresh = config.Distance_Threshold;
	optimize = config.Optimize_Coefficients;
	choose_image = config.image;
}

void pcl_segment::segmentPlane(pcl::PCLPointCloud2::Ptr cloud_)
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
	// seg.setMaxIterations(MaxIt);
    seg.setDistanceThreshold (DistThresh);
//    seg.setDistanceThreshold(0.01);
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

void pcl_segment::segmentEuclidean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_)
{
    pcl::VoxelGrid<PointT> vox;
    pcl::PointCloud<PointT>::Ptr cloud_filt (new pcl::PointCloud<PointT>);
    vox.setInputCloud(cloud_);
    vox.setLeafSize(0.01f, 0.01f, 0.01f);
    vox.filter(*cloud_filt);

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold (0.03);
    seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);

    int nr_points = (int) cloud_filt->points.size();
    while(cloud_filt->points.size() > 0.3* nr_points)
    {
        seg.setInputCloud (cloud_filt);
        seg.segment (*inliers, *coefficients);
        if(inliers->indices.size() == 0)
            ROS_INFO("No estimation for plane possible");

        pcl::ExtractIndices<PointT>  extract;
        extract.setInputCloud(cloud_filt);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);


        extract.setNegative(true);
        extract.filter(*cloud_filt);
        ROS_INFO_STREAM("Orig size: " << nr_points << ", size now: "  << cloud_filt->points.size() );
    }

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filt);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> extractor;
    extractor.setClusterTolerance(0.02);
    extractor.setMinClusterSize(100);
    extractor.setMaxClusterSize(25000);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud_filt);
    extractor.extract(cluster_indices);

    ROS_INFO_STREAM("Cluster indices size: " << cluster_indices.size());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filt->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
            pub_4.publish(cloud_cluster);

  }
}

void pcl_segment::segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_)
{
    pcl::PointCloud<PointT>::Ptr cloud_filt (new pcl::PointCloud<PointT>), cloud_filt2 (new pcl::PointCloud<PointT>), cloud_plane (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(new pcl::PointCloud<pcl::Normal>), cloud_norm2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cyl (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cyl (new pcl::PointIndices);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filt);


    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filt);
    ne.setKSearch(50);
    ne.compute(*cloud_norm);

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filt);
    seg.setInputNormals(cloud_norm);
    seg.segment (*inliers_plane, *coefficients_plane);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filt);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    extract.filter(*cloud_plane);

    pub_4.publish(cloud_plane);

    pcl::ExtractIndices<pcl::Normal> extract_norm;
    extract.setNegative(true);
    extract.filter(*cloud_filt2);
    extract_norm.setInputCloud(cloud_norm);
    extract_norm.setIndices(inliers_plane);
    extract_norm.setNegative(true);
    extract_norm.filter(*cloud_norm2);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations(10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud (cloud_filt2);
    seg.setInputNormals(cloud_norm2);
    seg.segment (*inliers_cyl, *coefficients_cyl);

    extract.setInputCloud(cloud_filt2);
    extract.setIndices(inliers_cyl);
    extract.setNegative(false);

    pcl::PointCloud<PointT>::Ptr cloud_cyl (new pcl::PointCloud<PointT> ());

    extract.filter(*cloud_cyl);
    if(cloud_cyl->points.empty())
        ROS_INFO("No cylinders found");
    else
        pub_3.publish(cloud_cyl);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_segment");
  ros::NodeHandle nh;
  pcl_segment pcl_object(nh);
  ros::spin();
  return 0;
}
