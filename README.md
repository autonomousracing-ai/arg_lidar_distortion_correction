# Lidar distortion correction

## General

Lidar distortion is caused by the movement of a Lidar sensor during the scanning process, e.g., once it is mounted on a driving vehicle.
There, single measurement points have different reference locations in space resulting in inconsistent data.
Measured points appear at wrong distance (straight line movement) and wrong direction (vehicle is turning), resulting in Lidar distortion.

No distortion           |  Distortion
:-------------------------:|:-------------------------:
![stand still](https://github.com/autonomousracing-ai/lidar_distortion_correction/figures/stand_still.png) | ![Lidar distortion](https://github.com/autonomousracing-ai/lidar_distortion_correction/figures/distortion.png)
*No distortion at stand still: raw point cloud in pink matches corrected point cloud in black* | *Distorted point cloud in pink and corrected point cloud in black from Lidar sensor on a moving vehicle.*

The distortion error is increased with faster movement, meaning higher linear or angular velocity.
Measurements from recent scanning positions exhibit less errors than the ones from the start of the scan.
Assuming the relative velocity between a vehicle and a measured object is 30 m/s and a single revolution of a LiDAR takes 100 ms, then the distortion from the first measured angle position to the last is 30 m/s * 0.1 s = 3 m.

However, the Lidar distortion correction presented can correct the distortion assuming constant known velocity and turn rate during measurement.
Additional inconsistency may be introduced by processing delays of applications using the resulting point cloud.
Once delays or at least a lower bound are known a priori, the delay can be anticipated by converting/extrapolating the measurement to future frames.
This increases performance and accuracy of applications relying on the point cloud.

The resulting pipeline: 
![Lidar distortion correction pipeline](https://github.com/autonomousracing-ai/lidar_distortion_correction/figures/distortion_correction_pipeline.png).
*Providing up-to-date Lidar data: distortion correction for the duration of the scan and the computation time of the correction itself as well as extrapolation of the point cloud considering application dependent time delays.*

More information can be found in [this paper](https://ieeexplore.ieee.org/document/9128372) [1].

[1] Renzler, T., Stolz, M., Schratter, M., and Watzenig, D. "Increased accuracy for fast moving LiDARS: Correction of distorted point clouds." 2020 IEEE International Instrumentation and Measurement Technology Conference (I2MTC). IEEE, 2020.


## ROS Node

The repository code contains a fully operational ROS node for Lidar distortion correction.
It requires a point cloud (sensor_msgs::PointCloud2) and odometry data (geometry_msgs::TwistStamped).
The point cloud may also be already preprocessed, e.g., filtered: measurement angles determining the amount of correction are calculated based on (x,y) information of the point cloud.
Angular and linear velocity are averaged between current and previous measurement.
The consideration of processing delays can be enabled/disabled by the parameter timing_correction.
