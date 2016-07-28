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
#include <pcl_tiago/pclConfig.h>


class pcl_test
{
public:
	pcl_test(ros::NodeHandle nh_);
	~pcl_test();
	void filter(pcl::PCLPointCloud2::Ptr cloud_);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	void dynam_CB(pcl_tiago::pclConfig &config, uint32_t level);
	double setLeafX, setLeafY, setLeafZ, MaxIt, DistThresh;

	pcl::PCLPointCloud2::Ptr cloud_filtered;
	ros::Subscriber sub;
  	ros::Publisher pub, pub_1, pub_2;
  	dynamic_reconfigure::Server<pcl_tiago::pclConfig> server;
  	dynamic_reconfigure::Server<pcl_tiago::pclConfig>::CallbackType f;

};