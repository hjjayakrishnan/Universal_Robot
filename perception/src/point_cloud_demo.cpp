/**
*
* @author Jayakrishnan HJ
* 
* @date May 2018
*
*/

#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/plane_segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  //perception::Cropper cropper;
  //Cropper cropper;
  
  //ros::Subscriber sub = nh.subscribe("cloud_in", 1, &Cropper::Callback, &cropper);
  ros::Publisher crop_pub =
        nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);  
   
  ros::Subscriber sub_crop = 
        nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
        
  ros::Publisher down_pub =
        nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  perception::Downsampler downsampler(down_pub); 
  
  ros::Subscriber sub_downsample = 
        nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
        
  ros::Publisher surface_pub =
        nh.advertise<sensor_msgs::PointCloud2>("surface_cloud", 1, true); 
  ros::Publisher above_surface_pub =
        nh.advertise<sensor_msgs::PointCloud2>("above_surface_cloud", 1, true); 
  ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("marker_cloud", 100);      
  perception::Segmenter segmenter(surface_pub,above_surface_pub,marker_pub);
  ros::Subscriber sub_segmenter = 
        nh.subscribe("downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter);
  
  ros::spin();
  return 0;
}
