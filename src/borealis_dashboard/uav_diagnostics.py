import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from uwb_msgs.msg import UUBmsg, UWBReading
from topic_visualizer import TopicVisualize
from odometry_visualizer import OdometryVisualizer
from button_service import ButtonService
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math

class UAV_Diagnostic(QObject):
    start_record_signal = qt.QtCore.pyqtSignal(bool)

    def __init__(self, layout, uwb_topic, odom_topic, uav_name, uwb_service, datafeed_service, target_topic):
        super(UAV_Diagnostic, self).__init__()
    
        # Attributes
        self.uav_name = uav_name
        self.uwb_topic = uwb_topic
        self.odom_topic = odom_topic
        self.target_topic = target_topic
        self.layout = layout

        self.uwb_started = False
        
        # Create service buttons
        self.uwb_button = ButtonService(uwb_service, "UWB", "UAV")
        self.datafeed_button = ButtonService(datafeed_service, "DataFeed", "UAV")

        # self.createLayout(self.layout, self.uav_name)
        self.createInterface()

    def createInterface(self):
        self.uwb_layout = TopicVisualize(self.uwb_topic, UUBmsg, "UWB")
        self.odometry_layout = TopicVisualize(self.odom_topic, Odometry, "Odometry")
        self.odometry_values_layout = OdometryVisualizer(self.target_topic, "Target")
        self.start_uwb_button = qt.QtWidgets.QPushButton("Start UWB")

        hLine1 = qt.QtWidgets.QFrame()
        hLine1.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine1.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        hLine2 = qt.QtWidgets.QFrame()
        hLine2.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine2.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        boxLayout = qt.QtWidgets.QVBoxLayout()
        boxLayout.addSpacing(20)

        vSpacer  = qt.QtWidgets.QSpacerItem(20, 40, qt.QtWidgets.QSizePolicy.Minimum, qt.QtWidgets.QSizePolicy.Expanding)
       
        boxLayout.addLayout(self.uwb_layout)
        boxLayout.addWidget(hLine1)
        boxLayout.addLayout(self.odometry_layout)
        boxLayout.addWidget(hLine2)
        boxLayout.addLayout(self.odometry_values_layout)
        boxLayout.addItem(vSpacer)
        boxLayout.addWidget(self.uwb_button)
        boxLayout.addWidget(self.datafeed_button)

        group = qt.QtWidgets.QGroupBox(self.uav_name)
        group.setStyleSheet("QGroupBox { border: 1px solid black; }")
        # group.setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 1px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top; }")
        group.setLayout(boxLayout)

        self.layout.addWidget(group)

    def deinit(self):
        pass

    def start(self, cmd):
        # Start / End UWB
        self.uwb_button.call(cmd)
        # Start / End Datafeed
        self.datafeed_button.call(cmd)

    def clear(self):
        self.uwb_left_layout.clear()
        self.uwb_right_layout.clear()
        self.odometry_layout.clear()
        
    
