import rospy, rostopic
import rospkg
import psutil
import subprocess
import shlex
import sys
import os
import datetime
import signal

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from topic_visualizer import TopicVisualize
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
#from playsound import playsound
from distance_between import DistanceBetween
from distance_between_posestamped import DistanceBetweenPoseStamped

class ControlCenter(QObject):
    
    def __init__(self, layout, callback):
        super(ControlCenter, self).__init__()
        """
            param layout: GUI Layout
            param callback: Experiment Start / End callback function that takes bool value as input
                                true - Start Experiment
                                false - End Experiment
        """
        # Attributes
        self.rviz = None
        self.path_visualizer = None
        self.target_publisher = None
        self.recorder = None
        self.recorder_args = []
        self.callback = callback

        # Calculate time zone offset
        time_offset = datetime.datetime.now() - datetime.datetime.utcnow()
        self.time_zone_offset = time_offset.total_seconds()

        self.visualization = False
        self.target_publishing = False
        self.record = False
        self.experiment_started = False
            
        # Get Parameters
        # self.command_topic = '/command_mode_inst'
        self.command_topic = '/hri_mode'
        # self.follow_me_command = rospy.get_param(rospy.get_name()+ '/follow_me_cmd', 'Follow')
        # self.gun_command = rospy.get_param(rospy.get_name()+ '/go_cmd', 'Go')
        self.human_sub_topic = rospy.get_param(rospy.get_name()+ '/human_cmd', '/hri_mode')
        self.bag_topics = rospy.get_param(rospy.get_name()+ '/recorded_topics', '-a')
        self.bag_directory = rospy.get_param(rospy.get_name()+ '/saved_directory', '~')
        self.gun_odom_topic = rospy.get_param(rospy.get_name()+ '/gun_odom_topic', '/borealis/command/pose') #TODO, ADD ONE DISPLAY FOR EACH DRONE, might not be needed since local pos already shown
        # Create GUI Layout
        self.layout = layout
        self.createLayout(self.layout)

        # Signal Connections
        self.start_visualization_button.clicked.connect(self.start_visualization)
        self.start_target_publisher_button.clicked.connect(self.start_target_publisher)
        self.gun_target_button.clicked.connect(self.select_gun_target)
        self.follow_me_button.clicked.connect(self.select_follow_me_target)
        self.record_button.clicked.connect(self.record_bag)
        self.start_button.clicked.connect(self.start)
        self.end_button.clicked.connect(self.end)

        # Start Publishers and Subscribers
        rospy.Subscriber(self.human_sub_topic, String, self.command_callback)
        self.human_command=None
        # self.mode=None
        rospy.Subscriber(self.gun_odom_topic, PoseWithCovarianceStamped, self.gun_odom_callback)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=100)

        rospy.Rate(2) # Run at lower rate to prevent bandwidth saturation
        
    

    def createLayout(self, layout_):

        ###### Sub Modules ######
        self.start_visualization_button = qt.QtWidgets.QPushButton("Start Visualization")
        self.start_target_publisher_button = qt.QtWidgets.QPushButton("Start Target Publisher")
        self.target_label = qt.QtWidgets.QLabel("")
        self.gun_pose_desc_label = qt.QtWidgets.QLabel("GunPose [x,y]: ")
        self.gun_pose_label = qt.QtWidgets.QLabel("")
        self.gun_target_button = qt.QtWidgets.QPushButton("Gun")
        self.follow_me_button = qt.QtWidgets.QPushButton("Follow")
        self.record_button = qt.QtWidgets.QPushButton("Record")

        subModules = qt.QtWidgets.QVBoxLayout()
        subModules.addSpacing(20)

        subModules.addWidget(self.start_visualization_button)
        subModules.addWidget(self.start_target_publisher_button)

        target = qt.QtWidgets.QHBoxLayout()
        self.target_label.setMinimumWidth(200)
        target.addWidget(self.target_label)
        target.addWidget(self.gun_target_button)
        target.addWidget(self.follow_me_button)
        targetGroup = qt.QtWidgets.QGroupBox()
        targetGroup.setLayout(target)
        targetGroup.setStyleSheet("QGroupBox {border: 0px solid black; }")
        subModules.addWidget(targetGroup)

        gun_target_pose = qt.QtWidgets.QHBoxLayout()
        self.gun_pose_desc_label.setMinimumWidth(200)
        self.gun_pose_label.setMinimumWidth(300)
        gun_target_pose.addWidget(self.gun_pose_desc_label)
        gun_target_pose.addWidget(self.gun_pose_label)
        gun_target_group = qt.QtWidgets.QGroupBox()
        gun_target_group.setLayout(gun_target_pose)
        gun_target_group.setStyleSheet("QGroupBox {border: 0px solid black; }")
        subModules.addWidget(gun_target_group)

        subModules.addWidget(self.record_button)

        subGroup = qt.QtWidgets.QGroupBox("Sub Modules")
        subGroup.setStyleSheet("QGroupBox { border: 1px solid black; }")

        # group.setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 1px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top; }")
        subGroup.setLayout(subModules)

        ###### Distance between Modules ######
        distanceModule = qt.QtWidgets.QVBoxLayout()
        distanceModule.addSpacing(10)
        distanceModule.addLayout(DistanceBetween("/UAV1PoseUWB", "/UAV2PoseUWB", "UAV1/UAV2", 1.5))
        distanceModule.addLayout(DistanceBetween("/UAV1PoseUWB", "/HumanPose", "UAV1/Human[UWB]", 1.5))
        distanceModule.addLayout(DistanceBetween("/UAV2PoseUWB", "/HumanPose", "UAV2/Human[UWB]", 1.5))
        distanceModule.addLayout(DistanceBetweenPoseStamped("/uav1/mavros/local_position/pose", "/HumanPose", "UAV1/Human", 1.5))
        distanceModule.addLayout(DistanceBetweenPoseStamped("/uav2/mavros/local_position/pose", "/HumanPose", "UAV2/Human", 1.5))
        distanceGroup = qt.QtWidgets.QGroupBox("Distance Values")
        distanceGroup.setStyleSheet("QGroupBox { border: 1px solid black; }")
        distanceGroup.setLayout(distanceModule)

        ###### Main Modules ######
        self.start_button = qt.QtWidgets.QPushButton("Start Experiment")
        self.end_button = qt.QtWidgets.QPushButton("End Experiment")
        self.experiment_name_field = qt.QtWidgets.QTextEdit()

        mainModule = qt.QtWidgets.QVBoxLayout()
        mainModule.addSpacing(20)
        mainModule.addWidget(qt.QtWidgets.QLabel("Experiment Name:"))
        mainModule.addWidget(self.experiment_name_field)
        mainModule.addWidget(self.start_button)
        mainModule.addWidget(self.end_button)
        mainGroup = qt.QtWidgets.QGroupBox("Main Control")
        mainGroup.setStyleSheet("QGroupBox { border: 1px solid black; }")
        mainGroup.setLayout(mainModule)

        vLayout = qt.QtWidgets.QVBoxLayout()
        vLayout.addSpacing(20)
        vLayout.addWidget(subGroup)
        vLayout.addWidget(distanceGroup)
        vLayout.addWidget(mainGroup)
        group = qt.QtWidgets.QGroupBox("Experiment Control")
        group.setStyleSheet("QGroupBox { border: 1px solid black; }")
        group.setLayout(vLayout)

        layout_.addWidget(group)

    def deinit(self):
        self.experiment_started = True
        self.end()

    def clear(self):
        pass
        
    def start(self):
        # Shutdown sub processes
        if self.visualization: self.start_visualization()
        if self.target_publishing: self.start_target_publisher()
        if self.record: self.record_bag()
        if not(self.callback is None):
            # Start Experiment
            self.callback(True)

        if self.experiment_started:
            # If an experiment is already running, rename bag file as incomplete
            source = os.path.join(self.bag_directory, self.bag_file_name + ".bag")
            destination = os.path.join(self.bag_directory, "incomplete_" + self.bag_file_name + ".bag") 
            os.rename(source, destination)   
            
        else:
            # Change Button State
            self.start_button.setStyleSheet("QPushButton { background: red }")
            self.start_button.setText("Restart Experiment")
            self.end_button.setStyleSheet("QPushButton { background: rgb(255, 255, 0) }")

            # Change status to experiment started
            self.experiment_started = True
        
        # Start / Restart Experiment
        # Start Visualization
        self.start_visualization()
        # Start Target Publisher
        self.start_target_publisher()
        # Start Bag Recorder
        self.record_bag()
        if not(self.callback is None):
            # Start Experiment
            self.callback(True)
                

    def end(self):
        if self.experiment_started:
            # Shutdown sub processes
            if self.visualization: self.start_visualization()
            if self.target_publishing: self.start_target_publisher()
            if self.record: self.record_bag()
            if not(self.callback is None):
                # End Experiment
                self.callback(False)

            # Change Button State
            self.start_button.setStyleSheet("")
            self.start_button.setText("Start Experiment")
            self.end_button.setStyleSheet("")

            # Change status to experiment not started
            self.experiment_started = False

    def command_callback(self, msg):
        # play sound only when there is an update
        #if (self.target_label.text != msg.data):
        #    if (msg.data == self.gun_command):
        #        playsound(os.path.join(rospkg.RosPack().get_path('borealis_dashboard'), 'media', 'GunMode.mp3'))
        #    elif (msg.data == self.follow_me_command):
        #        playsound(os.path.join(rospkg.RosPack().get_path('borealis_dashboard'), 'media', 'FollowMe.mp3'))
        #    else:
        #        playsound(os.path.join(rospkg.RosPack().get_path('borealis_dashboard'), 'media', 'Disabled.mp3'))
        self.human_command=msg # Copy command string to topic
        # self.target_label.setText(msg.data)

    def gun_odom_callback(self, msg):
        pose = "%.2f,  %.2f"%(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.gun_pose_label.setText(pose)

    def select_gun_target(self):
        # self.mode="Go"
        print("Switching to Go There")
        self.command_pub.publish("Go_There")
    
    def select_follow_me_target(self):
        # self.mode="Follow"
        print("Switching to Follow Me")
        self.command_pub.publish("Follow_Me")
    
    def select_human_target(self):
        # self.mode="Human"
        self.command_pub.publish(self.human_command)
    
    def start_target_publisher(self):
        self.target_publishing = not self.target_publishing
        if self.target_publishing:
            # Turn on Target Publisher
            self.start_target_publisher_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
            self.start_target_publisher_button.setText("Stop Target Publisher")

            # Launch Target Publisher
            # self.target_publisher = subprocess.Popen(['roslaunch', 'borealis_uav_target_publisher', 'uav_target_publisher.launch'], stdout=sys.stdout, stderr=sys.stderr)
            self.target_publisher = subprocess.Popen(['roslaunch', 'borealis_dashboard', 'control.launch'], stdout=sys.stdout, stderr=sys.stderr)

        else:
            # Turn off visualization
            self.start_target_publisher_button.setStyleSheet("")
            self.start_target_publisher_button.setText("Start Target Publisher")

            # End Visualization
            if not(self.target_publisher is None):
                self.target_publisher.terminate()
        
        # #rospy.spin()
        # if self.mode=="Human":
        #     self.command_pub.publish(self.human_command)
        # elif self.mode=="Go":
        #     self.command_pub.publish("Go")
        # elif self.mode=="Follow":
        #     self.command_pub.publish("Follow")
        rospy.sleep(1)

    def start_visualization(self):
        self.visualization = not self.visualization
        if self.visualization:
            # Turn on visualization
            self.start_visualization_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
            self.start_visualization_button.setText("Stop Visualization")

            # Launch Visualization
            self.rviz = subprocess.Popen(['roslaunch', 'borealis_dashboard', 'rviz.launch'], stdout=sys.stdout, stderr=sys.stderr)
            self.path_visualizer = subprocess.Popen(['roslaunch', 'pose_to_path', 'pose_to_path.launch'], stdout=sys.stdout, stderr=sys.stderr)

        else:
            # Turn off visualization
            self.start_visualization_button.setStyleSheet("")
            self.start_visualization_button.setText("Start Visualization")

            # End Visualization
            if not(self.rviz is None):
                self.rviz.terminate()
            if not(self.path_visualizer is None):
                self.path_visualizer.terminate()
        rospy.sleep(1)

    def terminate_ros_node(self, includes):
        # list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        # list_output = list_cmd.stdout.read()
        # retcode = list_cmd.wait()
        # assert retcode == 0, "List command returned %d" % retcode
        # for str in list_output.split("\n"):
        #     if (str.startswith(s)):
        #         os.system("rosnode kill " + str)

        for process in psutil.process_iter():
            if "record" in process.name() and set(includes).issubset(process.cmdline()):
                process.send_signal(subprocess.signal.SIGINT)
                rospy.loginfo("Process kill: pid %d"%process.pid)

    def record_bag(self):
        self.record = not self.record
        
        if self.record:
            # Turn on Recorder
            self.record_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
            self.record_button.setText("Stop Recording")

            # Start Bag Recorder
            args = ['rosbag', 'record']
            self.recorder_args =  []
            for topic in self.bag_topics:
                if topic[0] == '/':
                    topic = topic[1:]
                args.append("/" + topic)
                self.recorder_args.append("/" + topic)
            
            timeString = datetime.datetime.utcfromtimestamp(rospy.get_time() + self.time_zone_offset).strftime('%Y-%m-%d-%H-%M-%S')
            if self.experiment_name_field.toPlainText() == "":
                self.bag_file_name = timeString
            else:
                self.bag_file_name = self.experiment_name_field.toPlainText() + "_" + timeString
            directory = os.path.join(self.bag_directory, self.bag_file_name)

            args.append('-O')
            args.append(directory)

            self.recorder = subprocess.Popen(args, stdout=sys.stdout, stderr=sys.stderr)
        else:
             # Turn off Recorder
            self.record_button.setStyleSheet("")
            self.record_button.setText("Start Recording")

            # Stop Bag Recorder
            if not (self.recorder is None):
                self.terminate_ros_node(includes=self.recorder_args)
        rospy.sleep(1)

