import rospy, rostopic
import rospkg
import tf
import geometry_msgs

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64

# from importlib import import_module
# import sys

class DistanceBetweenPoseStamped(QFormLayout):
    odometry_signal = qt.QtCore.pyqtSignal(float, float) # x, y

    def __init__(self, topic, topic2, name=None, barrier=1.5):
        super(DistanceBetweenPoseStamped, self).__init__()
        
        # Attributes
        self.topic = topic
        self.topic2 = topic2
        self.barrier = barrier
        if name is None:
            name = topic

        self.x=0
        self.y=0
        self.z=0
        self.x2=0
        self.y2=0
        self.z2=0

        self.createLayout(name)

        # Subscribers
        self.rate = rostopic.ROSTopicHz(2)
        # rospy.Subscriber(self.topic, Odometry, self.rate.callback_hz, self.callback)
        rospy.Subscriber(self.topic, PoseStamped, self.callback)
        rospy.Subscriber(self.topic2, PoseWithCovarianceStamped, self.callback2)
        # rospy.Subscriber(self.topic, rospy.AnyMsg, self.callback)
        # rospy.Subscriber(self.topic2, rospy.AnyMsg, self.callback2)

        self.odometry_signal.connect(self.showOdometry)
        self.dist=math.sqrt( (self.x-self.x2)**2 + (self.y-self.y2)**2)
        self.odometry_signal.emit(self.dist, self.barrier)

        rospy.Rate(2) # Run at lower rate to prevent bandwidth saturation


    def createLayout(self, name):
        Desc = qt.QtWidgets.QLabel(name)
        self.data_label = qt.QtWidgets.QLabel("")
        self.addRow(Desc, self.data_label)

    def deinit(self):
        pass

    def clear(self):
        self.data_label.setText("")
        
    def callback(self, data):
        # assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
        # connection_header =  data._connection_header['type'].split('/')
        # ros_pkg = connection_header[0] + '.msg'
        # msg_type = connection_header[1]
        # # print 'Message type detected as ' + msg_type
        # msg_class = getattr(import_module(ros_pkg), msg_type)
        # msg = msg_class().deserialize(data._buff)

        # Check for different data types
        if data._type== "nav_msgs/Odometry" or data._type== "geometry_msgs/PoseWithCovarianceStamped":
            self.x=data.pose.pose.position.x
            self.y=data.pose.pose.position.y
            self.z=data.pose.pose.position.z

        elif data._type== "geometry_msgs/PoseStamped":
            self.x=data.pose.position.x
            self.y=data.pose.position.y
            self.z=data.pose.position.z

        else:
            print("Invalid message type")
        self.odometry_signal.emit(self.dist, self.barrier)
        

    def callback2(self, data):
        # Check for different data types
        if data._type== "nav_msgs/Odometry" or data._type== "geometry_msgs/PoseWithCovarianceStamped":
            self.x2=data.pose.pose.position.x
            self.y2=data.pose.pose.position.y
            self.z2=data.pose.pose.position.z

        elif data._type== "geometry_msgs/PoseStamped":
            self.x2=data.pose.position.x
            self.y2=data.pose.position.y
            self.z2=data.pose.position.z

        else:
            print("Invalid message type")
        self.odometry_signal.emit(self.dist, self.barrier)
    
    def showOdometry(self, dist, barrier):

        dist=math.sqrt( (self.x-self.x2)**2 + (self.y-self.y2)**2)

        if abs(dist) < barrier:
            self.data_label.setStyleSheet("QLabel { background: red }")
        elif abs(dist) >= barrier and abs(dist) < barrier + 1:
            self.data_label.setStyleSheet("QLabel { background: orange }")
        else:
            self.data_label.setStyleSheet("")

        self.data_label.setText("%.2f"%dist)