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
- nav_msgs/Odometry : /imu_odometry
- nav_msgs/Odometry : /camera_1/odom/sample
- nav_msgs/Odometry : /camera_2/odom/sample
- UUBmsg : /UAV1/UAV1_left
- UUBmsg : /UAV1/UAV1_right
- UUBmsg : /UAV2/UAV2_left
- UUBmsg : /UAV2/UAV2_right
- std_msgs/String : /command
  
#### Published Topics
- std_msgs/String : /command

#### Service Clients
std_srvs/SetBool : /human_ros_launcher/odometry
std_srvs/SetBool : /human_ros_launcher/fusion
std_srvs/SetBool : /uav1_ros_launcher/uwb
std_srvs/SetBool : /uav2_ros_launcher/uwb

### Note

Before running the dashboard, make sure you configure the "recorded_topics" and "saved_directory" parameters in config/parameters.yaml file. 
These parameters relate to the rosbag recording for the experiment.

