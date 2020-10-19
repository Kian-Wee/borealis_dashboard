import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from uwb_msgs.msg import UUBmsg, UWBReading
from topic_visualizer import TopicVisualize
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math

class UAV_Diagnostic(QObject):
    start_record_signal = qt.QtCore.pyqtSignal(bool)

    def __init__(self, layout, left_topic, right_topic, odom_topic, uav_name):
        super(UAV_Diagnostic, self).__init__()
        
        # Attributes
        self.uav_name = uav_name
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.odom_topic = odom_topic
        self.layout = layout

        self.record = False
        self.auto_follow = False
        
        # self.createLayout(self.layout, self.uav_name)
        self.createInterface()

    def createInterface(self):
        self.uwb_left_layout = TopicVisualize(self.left_topic, UUBmsg, "UWB_Left")
        self.uwb_right_layout = TopicVisualize(self.right_topic, UUBmsg, "UWB_Right")
        self.odometry_layout = TopicVisualize(self.odom_topic, Odometry, "Odometry")

        hLine1 = qt.QtWidgets.QFrame()
        hLine1.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine1.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        hLine2 = qt.QtWidgets.QFrame()
        hLine2.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine2.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        hLine3 = qt.QtWidgets.QFrame()
        hLine3.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine3.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        boxLayout = qt.QtWidgets.QVBoxLayout()
        boxLayout.addSpacing(20)
       
        boxLayout.addLayout(self.uwb_left_layout)
        boxLayout.addWidget(hLine1)
        boxLayout.addLayout(self.uwb_right_layout)
        boxLayout.addWidget(hLine2)
        boxLayout.addLayout(self.odometry_layout)

        group = qt.QtWidgets.QGroupBox(self.uav_name)
        group.setStyleSheet("QGroupBox { border: 1px solid black; }")
        # group.setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 1px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top; }")
        group.setLayout(boxLayout)

        self.layout.addWidget(group)

    def deinit(self):
        pass

    def clear(self):
        self.uwb_left_layout.clear()
        self.uwb_right_layout.clear()
        self.odometry_layout.clear()
        
    
