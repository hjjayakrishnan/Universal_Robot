#include "perception/plane_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "visualization_msgs/Marker.h"
#include "pcl/common/common.h"

#include <vector>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

perception::Segmenter::Segmenter() {}

perception::Segmenter::Segmenter(const ros::Publisher& surface_pub,
                                 const ros::Publisher& above_surface_pub,
                                 const ros::Publisher& marker_pub)
                                : surface_pub_(surface_pub),
                                 above_surface_pub_(above_surface_pub),
                                 marker_pub_(marker_pub){}

void perception::Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);
    ROS_INFO("Got into segmenter");
    PointCloudC::Ptr segmented_cloud(new PointCloudC());
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

// Segment the plane out of the point cloud
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
     pcl::PointIndices::Ptr indices_custom (new pcl::PointIndices);
     pcl::PointIndices::Ptr indices_not_surface (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    
    //pcl::PointIndices indices_internal;
    pcl::SACSegmentation<PointC> seg;
    seg.setOptimizeCoefficients(true);
    // Search for a plane perpendicular to some axis (specified below).
    //seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // Set the distance to the plane for a point to be an inlier.
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);

    // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
    /*Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(10.0));*/

    // coeff contains the coefficients of the plane:
    // ax + by + cz + d = 0
    //pcl::ModelCoefficients coeff;
    seg.segment(*indices, *coeff);

   // indices = &indices_internal;
   //seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices_custom->indices.push_back(i);
    }else{
      indices_not_surface->indices.push_back(i);
    }
  } 

    if (indices_custom->indices.size() == 0) {
      ROS_ERROR("Unable to find surface.");
      return;
      }  
     else
      ROS_INFO("Got indices!!!!");
       
 
  // Given these data types:
  //  PointCloudC::Ptr original_cloud(new PointCloudC);
  //  pcl::PointIndices indices;
    PointCloudC::Ptr subset_cloud(new PointCloudC);

    // Extract subset of original_cloud into subset_cloud:
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_custom);
    extract.setNegative(false);
    extract.filter(*subset_cloud);
  
  //publishing surface cloud
  
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*subset_cloud, msg_out);
    surface_pub_.publish(msg_out);
    
    
  //Extract subset of original_cloud into subset_cloud of objects
  PointCloudC::Ptr subset_cloud_above(new PointCloudC);

    // Extract subset of original_cloud into subset_cloud:
    pcl::ExtractIndices<PointC> extract_above;
    extract_above.setInputCloud(cloud);
    extract_above.setIndices(indices_not_surface);
    extract_above.setNegative(false);
    extract_above.filter(*subset_cloud_above);
  
  //publishing surface cloud
  
   // sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*subset_cloud_above, msg_out);
    above_surface_pub_.publish(msg_out);

  
  //publishing marker cloud
  
    visualization_msgs::Marker table_marker;
    table_marker.ns = "table";
    //table_marker.header.frame_id = "camera_link";
    table_marker.type = visualization_msgs::Marker::CUBE;
    perception::CreateBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
    table_marker.color.r = 1;
    table_marker.color.a = 0.8;
    marker_pub_.publish(table_marker);
}


void perception::CreateBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                             geometry_msgs::Pose* pose,
                                             geometry_msgs::Vector3* dimensions){
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    pose->position.x = (max_pt.x() + min_pt.x()) / 2;
    pose->position.y = (max_pt.y() + min_pt.y()) / 2;
    pose->position.z = (max_pt.z() + min_pt.z()) / 2;
    pose->orientation.w = 1;

    dimensions->x = max_pt.x() - min_pt.x();
    dimensions->y = max_pt.y() - min_pt.y();
    dimensions->z = max_pt.z() - min_pt.z();
}
