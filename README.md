# Borealis Experiment Dashboard

Borealis Project Diagnostic Dashboard RQT Plugin

Install script for dependencies available(setup.bash), ensure the workspace used is borealis_ws for successful automatic install

## Teaming Specific Branch
Implemented display for planner stages

## Dependencies

* ROS

* Python

* QT5

* uwb_msgs (Custom Message Type)

* pose_to_path (Fused Odometry visualization package)

* borealis_uav_target_publisher (Human odometry to UAV Target conversion package)

* mt_msgs (dso custom msgs it is inside src folder)
  
* re (regex, inside default python lib)
  
  

**Install** <br />

>      pip install PyQt5
>      catkin_make --only-pkg-with-deps borealis_dashboard

**Run** <br /> 

>      rqt --standalone borealis_dashboard

#### Subscribed Topics
These topics should be configured in the parameters.yaml file inside config folder

- human_foot_imu_topic : sensor_msgs/IMU
- human_odometry_topic : nav_msgs/Odometry
- uav1_odometry_topic : nav_msgs/Odometry
- uav1_target_topic : nav_msgs/Odometry
- uav2_odometry_topic : nav_msgs/Odometry
- uav2_target_topic : nav_msgs/Odometry
- human_uwb_topic : UUBmsg
- uav2_uwb_topic : UUBmsg
- uav1_uwb_topic : UUBmsg
- command_topic: std_msgs/String

#### Service Clients
These services should be configured in the parameters.yaml file inside config folder

- human_odometry_enable_service : std_srvs/SetBool
- human_odometry_fusion_enable_service : std_srvs/SetBool
- human_glove_enable_service : std_srvs/SetBool
- human_gun_enable_service : std_srvs/SetBool
- human_hri_enable_service : std_srvs/SetBool
- human_drone_yaw_control_enable_service : std_srvs/SetBool
- human_uwb_enable_service : std_srvs/SetBool
- uav1_uwb_enable_service : std_srvs/SetBool
- uav1_datafeed_enable_service : std_srvs/SetBool
- uav2_uwb_enable_service : std_srvs/SetBool
- uav2_datafeed_enable_service : std_srvs/SetBool
  
#### Published Topics
- command_topic: std_msgs/String

#### Parameters
- recorded_topics : String List (List of topics to be recorded)
- saved_directory : Path (ROS Bag saving directory) 
- follow_me_cmd : String
- go_cmd : String

### Note

Before running the dashboard, make sure you configure the "recorded_topics" and "saved_directory" parameters in config/parameters.yaml file. 
These parameters relate to the rosbag recording for the experiment.
