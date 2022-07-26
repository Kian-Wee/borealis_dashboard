Header header 			        # header for msg tracking

uint64 groupId                  # group id of task (determines task communications namespace)
int8 type			            # UNDEFINED = 0; GO_THERE = 1; FOLLOW_ME = 2; DISTRACT_TARGET = 3;
uint64 numAgents                # number of agents involved in task
string[] agents                 # namespaces of agents
geometry_msgs/Pose[] targets    # target pose