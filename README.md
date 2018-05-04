# Universal_Robot
Independent study on designing a perception pipeline to compute grasp poses and mainpulate using move-it for the UR5 robot.  
youtube demo:https://www.youtube.com/watch?v=CdeaKv9TQwI

### Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites



* ROS and PCL 
* GPD package https://github.com/atenpas/gpd
* Universal Robot package https://github.com/ros-industrial/universal_robot
* UR modern driver https://github.com/ThomasTimm/ur_modern_driver


### Installing

Clone both the perception and UR5_package(along with all the pre-requisites) to your catkin workspace and build them using

```
catkin_make
```


### Usage

The following launch file will help ROS's parameter server has information on the UR5 robot.

```
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ROBOT_IP_ADDRESS
```
To use move-it for the manipulation planning launch the following: It will also open the RVIZ visualizer. Open the jay_is rviz config
in the visualization folder.
```
roslaunch UR5_package ur5_launch_jay.launch
```
To start the camera,
```
roslaunch openni2_launch openni2.launch
```
To start the perception pipeline 
```
rosrun perception point_cloud_demo cloud_in:=/cara/depth_registered/points
```

The perception window can be cropped at run time by using the following command:

Eg.
```
rosparam set crop_max_x 0.9
rosparam set crop_min_x 0.9
rosparam set crop_max_y 0.9
rosparam set crop_min_y 0.9
rosparam set crop_max_z 0.9
rosparam set crop_min_z 0.9
```
To start the Grasp pose detection: 
```
roslaunch gpd tutorial1.launch
```

To start the manipulation based on grasping:

```
rosrun UR5_package grasp_expt.py
```
### Contributing

Please submit a pull request. 
