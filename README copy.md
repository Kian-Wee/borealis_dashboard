# Boreolis Experiment Dashboard

Boreolis Project Diagnostic Dashboard RQT Plugin

## Dependencies
* ROS
* Python
* QT5
* uwb_msgs (Custom Message Type)
* pose_to_path (Fused Odometry visualization package)
* ros_launcher (Required on client computers)
  
  
**Install** <br /> 
>      catkin_make --only-pkg-with-deps borealis_dashboard

**Run** <br /> 
>      rqt --standalone borealis_dashboard

#### Subscribed Topics
- sensor_msgs/IMU : /footIMU/IMU
- sensor_msgs/IMU : /waistIMU/IMU
- nav_msgs/Odometry : /imu_odometry
- UUBmsg : /UUB1
- UUBmsg : /UUB2
- UUBmsg : /UUB3
  
#### Published Topics
- None

### Note

Before running the dashboard, make sure you configure the "recorded_topics" and "saved_directory" parameters in config/parameters.yaml file. 
These parameters relate to the rosbag recording for the experiment.

