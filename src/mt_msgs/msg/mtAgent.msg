string name                                         # namespace of agent
int8 task			                                # UNDEFINED = 0; GO_THERE = 1; FOLLOW_ME = 2; DISTRACT_TARGET = 3;
geometry_msgs/PoseWithCovariance systemPose         # system pose
geometry_msgs/TwistWithCovariance systemTwist       # system twist
geometry_msgs/PoseStamped localPose                 # local pose
std_msgs/Bool taskAllocated                         # true if agent has allocated itself a task