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


class pcl_segment
{
public:
	pcl_segment(ros::NodeHandle nh_);
	~pcl_segment();
	void filter(pcl::PCLPointCloud2::Ptr cloud_);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    void dynam_CB(pcl_tiago_tutorials::pclConfig &config, uint32_t level);
	double setLeafX, setLeafY, setLeafZ, MaxIt, DistThresh;
	bool optimize;
	int choose_image;

	pcl::PCLPointCloud2::Ptr cloud_filtered;
	ros::Subscriber sub;
  	ros::Publisher pub, pub_1, pub_2;
    dynamic_reconfigure::Server<pcl_tiago_tutorials::pclConfig> server;
    dynamic_reconfigure::Server<pcl_tiago_tutorials::pclConfig>::CallbackType f;

};
