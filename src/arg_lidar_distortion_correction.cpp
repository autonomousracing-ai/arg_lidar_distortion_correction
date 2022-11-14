
#include "arg_lidar_distortion_correction/arg_lidar_distortion_correction.h"

using ::ros::NodeHandle;
using namespace std;


ARGLidarDistortionCorrection::ARGLidarDistortionCorrection()
        : node_initialized_(false),
          system_ready_(false),
          rate_(10.0)
{
}


void ARGLidarDistortionCorrection::run()
{
    if (!node_initialized_)
    {
        ROS_FATAL("ARGLidarDistortionCorrection is not initialized. Shutdown.");
        return;
    }
    ros::spin();

}

// Initialize node
bool ARGLidarDistortionCorrection::init() {

  if ( readConfig())
  {
    node_initialized_ = true;
  }

  // create publishers
  pcd_undistorted_pub_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>("points_out", 1);

  // create subscribers
  if ( !OUSTER_MODE ) {
    pcd_in_sub_ = node_handle_.subscribe("points_in", 1,
        &ARGLidarDistortionCorrection::pcdCallback,
        this, ros::TransportHints().tcpNoDelay(true));
  }
  else {
    pcd_in_sub_ = node_handle_.subscribe("points_in", 1,
        &ARGLidarDistortionCorrection::pcdOusterCallback,
        this, ros::TransportHints().tcpNoDelay(true));
    cout << "OUSTER MODE" << endl;
  }



  twist_in_sub_ = node_handle_.subscribe("twist_in", 1,
      &ARGLidarDistortionCorrection::vehicleTwistCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  

  timer_ = node_handle_.createTimer(ros::Duration(1.0 / rate_),
                                        &ARGLidarDistortionCorrection::process, this);

  return node_initialized_;
}



bool ARGLidarDistortionCorrection::readConfig()
{
    ros::NodeHandle priv_nh("~");

    // Config arguments
    priv_nh.param<bool>("timing_correction", timing_correction_, false);
    priv_nh.param<bool>("ouster_mode", OUSTER_MODE, false);
    priv_nh.param<int>("rotation_frequency", rotation_frequency_, 0);

    

    return true;
}


void ARGLidarDistortionCorrection::process(const ros::TimerEvent &)
{
  ros::Time current_time = ros::Time::now();
}


void ARGLidarDistortionCorrection::pcdCallback(const sensor_msgs::PointCloud2 &pcd_in)
{
  pcd_in_ = pcd_in;

  static ros::Time prev_pcd_time = pcd_in.header.stamp;
  if ( rotation_frequency_ > 0) {
    delta_t_ = 1.0 / rotation_frequency_;
  }
  else {
    delta_t_ = pcd_in.header.stamp.toSec() - prev_pcd_time.toSec();
    cout << "dt: " << delta_t_ << endl;
  }

  prev_pcd_time = pcd_in.header.stamp;


  static std::vector<double> omega (2);
  static std::vector<double> v (2);

  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(pcd_in_, scan);

  //read pointcloud parameters
  int pointcloud_width = pcd_in_.width;
  int pointcloud_height = pcd_in_.height;

  //calculate vehicle's linear velocity, also keep the previous value.
  double vehicle_v = sqrt(pow(twist_.twist.linear.x, 2) + pow(twist_.twist.linear.y, 2));
  v.at(1) = v.at(0);
  v.at(0) = vehicle_v;

  //calculate vehicle's angular velocity, also keep the previous value.
  double angular_vz = twist_.twist.angular.z;
  omega.at(1) = omega.at(0);
  omega.at(0) = angular_vz;

  //average linear and angular velocity.
  double v_mean = (v.at(0) + v.at(1))/2;
  double omega_mean = (omega.at(0) + omega.at(1))/2;

  //counter to address the correct value. Ouster does not store values in rows and columns. Thus .at(column,row) or
  //.at(n), where n = column*width + row does not work!
  //Columns are stored row-wise, meaning the first row contains the width/height first columns.
  //Accessing the element can be achieved by .at(n) where n = element_cnt + j.
  
  double scan_x, scan_y, scan_z;
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = scan.points.begin(); it != scan.points.end(); it++ ) {
    
    //read point coordinates
    scan_x = it->x;
    scan_y = it->y;
    scan_z = it->z;
    double alpha = atan2(scan_y, scan_x);
    if(alpha < 0){
      alpha = alpha + 2*M_PI;
    }

    //define the position and the orientation of the new coordinate system
    double theta_i = 0*(M_PI/180);
    double x_i = 0;
    double y_i = 0;
    
    //delta angle dependent on the current alpha
    double delta_theta_alpha = omega_mean * delta_t_ * (alpha/(2*M_PI));
    double theta_alpha = theta_i - delta_theta_alpha;

    //correction in x and y direction dependent on vehicle's speed.
    double x_alpha = x_i + delta_t_ * v_mean * (alpha/(2*M_PI)) * cos(theta_i - delta_theta_alpha/2);
    double y_alpha = y_i + delta_t_ * v_mean * (alpha/(2*M_PI)) * sin(theta_i - delta_theta_alpha/2);

      //perform rotation
      double scan_x_rot = scan_x * cos(theta_alpha) - scan_y * sin(theta_alpha);
      double scan_y_rot = scan_x * sin(theta_alpha) + scan_y * cos(theta_alpha);
      double scan_z_rot = scan_z;

      //overwrite old values with corrected values.
      it->x = (x_alpha + scan_x_rot);
      it->y = (y_alpha + scan_y_rot);
      it->z = 0 + scan_z_rot;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan_ptr, msg);

  msg.header = pcd_in_.header;
  if ( timing_correction_) {
    msg.header.stamp = msg.header.stamp + ros::Duration(delta_t_);
  }
  pcd_undistorted_pub_.publish(msg);
}


void ARGLidarDistortionCorrection::pcdOusterCallback(const sensor_msgs::PointCloud2 &pcd_in)
{
  pcd_in_ = pcd_in;
  
  static ros::Time prev_pcd_time = pcd_in.header.stamp;
  if ( rotation_frequency_ > 0) {
    delta_t_ = 1.0 / rotation_frequency_;
  }
  else {
    delta_t_ = pcd_in.header.stamp.toSec() - prev_pcd_time.toSec();
    cout << "dt: " << delta_t_ << endl;
  }

  prev_pcd_time = pcd_in.header.stamp;

  static std::vector<double> omega (2);
  static std::vector<double> v (2);

  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(pcd_in_, scan);

  //read pointcloud parameters
  int pointcloud_width = pcd_in_.width;
  int pointcloud_height = pcd_in_.height;

  //calculate vehicle's linear velocity, also keep the previous value.
  double vehicle_v = sqrt(pow(twist_.twist.linear.x, 2) + pow(twist_.twist.linear.y, 2));
  v.at(1) = v.at(0);
  v.at(0) = vehicle_v;

  //read vehicle's angular velocity, also keep the previous value.
  double angular_vz = twist_.twist.angular.z;
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

  msg.header = pcd_in_.header;
  if ( timing_correction_) {
    msg.header.stamp = msg.header.stamp + ros::Duration(delta_t_);
  }
  pcd_undistorted_pub_.publish(msg);
}


void ARGLidarDistortionCorrection::vehicleTwistCallback(const geometry_msgs::TwistStamped &twist)
{
  twist_ = twist;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arg_lidar_distortion_correction_node");

  ARGLidarDistortionCorrection arg_lidar_distortion_correction_node;
  if (arg_lidar_distortion_correction_node.init()) {
    arg_lidar_distortion_correction_node.run(); 
  }
  else {
    ROS_FATAL_STREAM("arg_lidar_distortion_correction_node initialization failed. Shutdown.");
  }

  return 0;
}
