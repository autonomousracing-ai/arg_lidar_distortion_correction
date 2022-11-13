//
// Created by rtobi on 05.08.19.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>

ros::Publisher distortion_correction_points_pub;



double delta_t_;

static void scan_imu_odom_callback(const sensor_msgs::PointCloud2::ConstPtr& input_pcl2,
        const geometry_msgs::TwistStamped::ConstPtr& input_twist){

  static std::vector<double> omega (2);
  static std::vector<double> v (2);

  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input_pcl2, scan);

  //read pointcloud parameters
  int pointcloud_width = input_pcl2->width;
  int pointcloud_height = input_pcl2->height;

  //calculate vehicle's linear velocity, also keep the previous value.
  double vehicle_v = sqrt(pow(input_twist->twist.linear.x, 2) + pow(input_twist->twist.linear.y, 2));
  v.at(1) = v.at(0);
  v.at(0) = vehicle_v;

  //read vehicle's angular velocity, also keep the previous value.
  double angular_vz = input_twist->twist.angular.z;
  omega.at(1) = omega.at(0);
  omega.at(0) = angular_vz;

  //average linear and angular velocity.
  double v_i = (v.at(0) + v.at(1))/2;
  double omega_i = (omega.at(0) + omega.at(1))/2;
  //reduce velocity as real travelled distance is larger than the assumed path between two pointcloud measurements
  v_i = v_i*sin((omega_i*delta_t_)/2)/(omega_i*delta_t_/2);

  //counter to address the correct value. Ouster does not store values in rows and columns. Thus .at(column,row) or
  //.at(n), where n = column*width + row does not work!
  //Columns are stored row-wise, meaning the first row contains the width/height first columns.
  //Accessing the element can be achieved by .at(n) where n = element_cnt + j.
  int element_cnt = 0;
  for(int i = 0; i < pointcloud_width; i++){

    //for each new ray position, calculate a new alpha (azimuth). Ouster starts at 360° and finishes at 0° in clockwise
    //direction. Thus the angle is subtracted from 2pi.
    double alpha = 2*M_PI - (double(i)/double(pointcloud_width))*2*M_PI;

    //define the position and the orientation of the new coordinate system
    double theta_i = 0*(M_PI/180);
    double x_i = 0;
    double y_i = 0;

    //delta angle dependent on the current alpha
    double delta_theta_alpha = omega_i * delta_t_ * (alpha/(2*M_PI));
    double theta_alpha = theta_i - delta_theta_alpha;

    //correction in x and y direction dependent on vehicle's speed.
    double x_alpha = x_i + delta_t_*v_i*(alpha/(2*M_PI))*cos(theta_i - delta_theta_alpha/2);
    double y_alpha = y_i + delta_t_*v_i*(alpha/(2*M_PI))*sin(theta_i - delta_theta_alpha/2);

    for(int j = 0; j < pointcloud_height; j++){

      //skip zero points
      if(scan.points.at(element_cnt+j).x == 0 && scan.points.at(element_cnt+j).y == 0
        && scan.points.at(element_cnt+j).z == 0){
        continue;
      }

      //read point coordinates
      double scan_x = scan.points.at(element_cnt+j).x;
      double scan_y = scan.points.at(element_cnt+j).y;
      double scan_z = scan.points.at(element_cnt+j).z;

      //perform rotation
      double scan_x_rot = scan_x * cos(theta_alpha) - scan_y * sin(theta_alpha);
      double scan_y_rot = scan_x * sin(theta_alpha) + scan_y * cos(theta_alpha);
      double scan_z_rot = scan_z;

      //overwrite old values with corrected values.
      scan.points.at(element_cnt+j).x = (x_alpha + scan_x_rot);
      scan.points.at(element_cnt+j).y = (y_alpha + scan_y_rot);
      scan.points.at(element_cnt+j).z = 0 + scan_z_rot;
    }
    //increase the counter as pointcloud_height elements were corrected successfully.
    element_cnt = element_cnt + pointcloud_height;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan_ptr, msg);

  msg.header = input_pcl2->header;
  distortion_correction_points_pub.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "distortion_correction");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int rotation_frequency = 0;
  private_nh.param<int>("rotation_frequency", rotation_frequency);

  if (rotation_frequency > 0) {
    delta_t_ = 1.0 / rotation_frequency;
  }
  else {
    delta_t_ = 0.05;
  }
  
  // Publishers
  distortion_correction_points_pub = nh.advertise<sensor_msgs::PointCloud2>("points_out", 10);

  // Subscribers
  message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub(nh, "points_in", 1);
  message_filters::Subscriber<geometry_msgs::TwistStamped> v_sub(nh, "twist_in", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::TwistStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), scan_sub, v_sub);
  sync.registerCallback(boost::bind(&scan_imu_odom_callback, _1, _2));

  ros::spin();

  return 0;
}
