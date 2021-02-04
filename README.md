# Boreolis Diagnostic Dashboard

Boreolis Project Diagnostic Dashboard RQT Plugin

## Dependencies
* ROS
* Python
* QT5
* uwb_msgs (Custom Message Type)
  
  
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


