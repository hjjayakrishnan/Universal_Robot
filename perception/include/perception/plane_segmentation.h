#ifndef PLANE_SEGMENTATION_H
#define PLANE_SEGMENTATION_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace perception {

void CreateBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                         geometry_msgs::Pose* pose,
                         geometry_msgs::Vector3* dimensions);
                         
class Segmenter {
 public:
  Segmenter();
  Segmenter(const ros::Publisher& surface_pub_,
            const ros::Publisher& above_surface_pub_,
            const ros::Publisher& marker_pub_);
  void Callback(const sensor_msgs::PointCloud2& msg);
  

 private:
 ros::Publisher surface_pub_;
 ros::Publisher above_surface_pub_;
 ros::Publisher marker_pub_;
};
}  // namespace perception

#endif
