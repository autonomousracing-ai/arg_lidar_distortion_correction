#ifndef _ARG_LIDAR_DISTORTION_CORRECTION_H_
#define _ARG_LIDAR_DISTORTION_CORRECTION_H_

//
// Created by rtobi on 05.08.19.
// Modification markusschratter 12.11.22
//

#include <cmath>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>





class ARGLidarDistortionCorrection {
  
  // Attributes
  bool system_ready_;
  bool node_initialized_;
  double rate_;

  double delta_t_;
  bool timing_correction_ = false;
  int rotation_frequency_ = 0;

  bool OUSTER_MODE = false;

  sensor_msgs::PointCloud2 pcd_in_;
  geometry_msgs::TwistStamped twist_;

  // Node handle
  ros::NodeHandle node_handle_;
  ros::Timer timer_;

  // Published topics
  ros::Publisher pcd_undistorted_pub_;

  // Subsribed topics
  ros::Subscriber pcd_in_sub_;
  ros::Subscriber twist_in_sub_;


  // Methods
  bool readConfig();

  void process(const ros::TimerEvent &);

  // Callbacks
  void pcdCallback(const sensor_msgs::PointCloud2 &pcd_in);
  void pcdOusterCallback(const sensor_msgs::PointCloud2 &pcd_in);

  void vehicleTwistCallback(const geometry_msgs::TwistStamped &twist);

  
public:
    ARGLidarDistortionCorrection();

    bool init();

    void run();

};

#endif // _ARG_LIDAR_DISTORTION_CORRECTION_H_
