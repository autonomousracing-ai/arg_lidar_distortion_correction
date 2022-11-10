# Lidar distortion correction

## General

Lidar distortion is caused by the movement of a Lidar sensor during the scanning process, e.g., once it is mounted on a driving vehicle.
There, single measurement points have different reference locations in space resulting in inconsistent data.
Measured points appear at wrong distance (straight line movement) and wrong direction (vehicle is turning), resulting in Lidar distortion.

No distortion           |  Distortion
:-------------------------:|:-------------------------:
![stand still](https://github.com/autonomousracing-ai/lidar_distortion_correction/blob/main/figures/stand_still.png) | ![Lidar distortion](https://github.com/autonomousracing-ai/lidar_distortion_correction/blob/main/figures/distortion.png)
*No distortion at stand still: raw point cloud in pink matches corrected point cloud in black.* | *Distorted point cloud in pink and corrected point cloud in black from Lidar sensor on a moving vehicle.*

The distortion error is increased with faster movement, meaning higher linear or angular velocity.
Measurements from recent scanning positions exhibit less errors than the ones from the start of the scan.
Assuming the relative velocity between a vehicle and a measured object is 30 m/s and a single revolution of a LiDAR takes 100 ms, then the distortion from the first measured angle position to the last is 30 m/s * 0.1 s = 3 m.

However, the Lidar distortion correction presented can correct the distortion assuming constant known velocity and turn rate during measurement.
Additional inconsistency may be introduced by processing delays of applications using the resulting point cloud.
Once delays or at least a lower bound are known a priori, the delay can be anticipated by converting/extrapolating the measurement to future frames.
This increases performance and accuracy of applications relying on the point cloud.

![Lidar distortion correction pipeline](https://github.com/autonomousracing-ai/lidar_distortion_correction/blob/main/figures/distortion_correction_pipeline.png)

*Providing up-to-date Lidar data: distortion correction for the duration of the scan and the computation time of the correction itself as well as extrapolation of the point cloud considering application dependent time delays.*

More information can be found in [this paper](https://ieeexplore.ieee.org/document/9128372) [1].

[1] Renzler, T., Stolz, M., Schratter, M., and Watzenig, D. "Increased accuracy for fast moving LiDARS: Correction of distorted point clouds." 2020 IEEE International Instrumentation and Measurement Technology Conference (I2MTC). IEEE, 2020.


## Simple example

We assume that a 360° scanning LiDAR sensor (
$\alpha_{\text{start}} = 0°$, 
$\alpha_{\text{end}}=360°$
) starts with a single scan process while having a heading of $\Theta_{i-1} = 0°$ at point $S$.
Till the end of the scan process, the LiDAR moves laterally and angular along a quarter-circle (
$b$
) with a radius $r=10$.
At the end of the scan, the sensor reaches position $E$ and has a heading of $\Theta_{i} = 90°$.
The sensor was able to measure a point $P$ from position $Z$ under a angle of $\alpha = 180°$, while having a heading of $\Theta_\alpha = 45°$.
The LiDAR records $P$ at 

$$\prescript{\alpha}{\alpha} r_{M} = \begin{bmatrix} -4.137 & 0 & 0 \end{bmatrix}^T.$$

![Lidar distortion example](https://github.com/autonomousracing-ai/lidar_distortion_correction/blob/main/figures/distortion_example.png)

At position $E$, once the whole point cloud is available, the distortion correction needs to be applied: the recorded coordinates of $P$ are not valid anymore.
All in the following steps used equations are explained in detail in our paper.
First we calculate the correction factor $c$

$$c = 1-\left|\frac{\alpha-\alpha_{\text{start}}}{\alpha_{\text{end}} - \alpha_{\text{start}}}\right| = 1 - \left|\frac{\pi-0}{2\pi-0}\right| = 0.5.$$

Afterwards, we calculate the angle $\delta$

$$ \delta = \frac{\Theta_{i} - \Theta_{\alpha}}{2} = \frac{\pi}{8}.$$

With the angle, the rotation matrix ${}^i R_\alpha$ can be calculated:

$$
	{}^i R_\alpha = \begin{bmatrix}
	cos(2\delta) & sin(2\delta) & 0 \\
	-sin(2\delta) & cos(2\delta) & 0 \\
	0 & 0 & 1
	\end{bmatrix} = \begin{bmatrix}
	\frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}} & 0 \\
	-\frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}} & 0 \\
	0 & 0 & 1
	\end{bmatrix}.
$$

Further, the displacement $\prescript{i}{i} r_\alpha$ is calculated

$$
	\prescript{i}{i} r_\alpha = c \cdot b 
	\begin{bmatrix} 
	-cos(\delta) \\ 
	sin(\delta) \\ 
	0
	\end{bmatrix} =
	0.5 \cdot 2\pi r \cdot \frac{\frac{\pi}{2}}{2\pi}
	\begin{bmatrix} 
	-cos(\frac{\pi}{8}) \\ 
	sin(\frac{\pi}{8}) \\ 
	0
	\end{bmatrix} =
	\begin{bmatrix} 
	-7.256 \\ 
	3.006 \\ 
	0
	\end{bmatrix}.	
$$

Finally, the corrected position of point $P$, seen from point $T$, can be calculated:

$$
	\prescript{i}{i} r_M = \prescript{i}{i} r_\alpha + {}^i R_\alpha \cdot {}^\alpha _\alpha r_M =
	\begin{bmatrix} 
	-7.256 \\ 
	3.006 \\ 
	0
	\end{bmatrix} + 
	\begin{bmatrix}
	\frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}} & 0 \\
	-\frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}} & 0 \\
	0 & 0 & 1
	\end{bmatrix} \cdot 
	\begin{bmatrix} 
	-4.137 \\
	0 \\ 
	0 
	\end{bmatrix} =
	\begin{bmatrix} 
	-10.181 \\ 
	5.931 \\ 
	0 
	\end{bmatrix}
$$

The corrected position is a good approximation but not exact.
More information can be found in our paper.
Accuracy can be increased by using

$$\prescript{i}{i} r_\alpha = c \cdot b \cdot \frac{|sin(\delta)|}{|\delta|}.$$

Applying again the last two steps, results in the coordinates of 

$$\prescript{i}{i} r_M = 
\begin{bmatrix}
-9.996 \\ 
5.854 \\ 
0
\end{bmatrix}.$$

Remaining deviations smaller than 0.01 result from rounding to the third decimal place throughout every calculation step.


## ROS Node

The repository code contains a fully operational ROS node for Lidar distortion correction.
It requires a point cloud (sensor_msgs::PointCloud2) and odometry data (geometry_msgs::TwistStamped).
The point cloud may also be already preprocessed, e.g., filtered: measurement angles determining the amount of correction are calculated based on (x,y) information of the point cloud.
Angular and linear velocity are averaged between current and previous measurement.
The consideration of processing delays can be enabled/disabled by the parameter timing_correction.

Terminal 1 - Execution:
```
$ source /opt/ros/melodic/setup.bash
$ mkdir ~/lidar_distortion_ws
$ cd ~/lidar_distortion_ws
$ catkin_make
$ cd src
$ git clone https://github.com/autonomousracing-ai/lidar_distortion_correction
$ catkin_make
$ source devel/setup.bash
$ roslaunch arg_distortion_correction arg_distortion_correction_general.launch lidar_localization:=true
```

Terminal 2 - Data:
```
$ source /opt/ros/melodic/setup.bash
$ rosbag play ~/path_to_data/2022-11-09-19-54-24.bag
```

Terminal 3 - Visualization:
```
$ source /opt/ros/melodic/setup.bash
$ rviz -d ~/ros/lidar_distortion_ws/src/lidar_distortion_correction/rviz/rviz_config.rviz 
```
