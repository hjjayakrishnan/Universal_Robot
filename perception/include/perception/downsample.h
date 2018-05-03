/**
*
* @author Jayakrishnan HJ
* 
* @date May 2018
*
*/

#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
class Downsampler {
 public:
  Downsampler();
  Downsampler(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
 ros::Publisher pub_;
};
}  // namespace perception

#endif
