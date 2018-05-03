#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

perception::Cropper::Cropper() {}

perception::Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

void perception::Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
 // ROS_INFO("Got point cloud with %ld points", cloud->size());
  
  //cropping
  //PointCloudC::Ptr cropped_cloud(new PointCloudC());
  //Eigen::Vector4f min_pt(0.3, -1, 0.5, 1);
  //Eigen::Vector4f max_pt(0.9, 1, 1.5, 1);
  
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  crop.filter(*cropped_cloud);
 // ROS_INFO("Cropped to %ld points", cropped_cloud->size());
  
  
  //publishing
  
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  pub_.publish(msg_out);
}
