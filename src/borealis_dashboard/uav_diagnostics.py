import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from uwb_msgs.msg import UUBmsg, UWBReading
from topic_visualizer import TopicVisualize
from odometry_visualizer import OdometryVisualizer
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math

class UAV_Diagnostic(QObject):
    start_record_signal = qt.QtCore.pyqtSignal(bool)

    def __init__(self, layout, left_topic, right_topic, odom_topic, uav_name, uwb_service, target_topic):
        super(UAV_Diagnostic, self).__init__()
        
        # Attributes
        self.uav_name = uav_name
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.odom_topic = odom_topic
        self.target_topic = target_topic
        self.layout = layout

        self.uwb_started = False
        
        # self.createLayout(self.layout, self.uav_name)
        self.createInterface()

        # Signal Connections
        self.start_uwb_button.clicked.connect(self.start_uwb)

        # Services
        self.uwb_service = None
        try:
            rospy.wait_for_service(uwb_service, timeout=3)
            self.uwb_service = rospy.ServiceProxy(uwb_service, SetBool)
        except Exception as e:
            self.start_uwb_button.setStyleSheet("QPushButton { background: red }")
            self.start_uwb_button.setText("UWB: Error")
            rospy.logwarn ("UAV Diagnostics : Exception:" + str(e))

    def createInterface(self):
        self.uwb_left_layout = TopicVisualize(self.left_topic, UUBmsg, "UWB_Left")
        self.uwb_right_layout = TopicVisualize(self.right_topic, UUBmsg, "UWB_Right")
        self.odometry_layout = TopicVisualize(self.odom_topic, Odometry, "Odometry")
        self.odometry_values_layout = OdometryVisualizer(self.target_topic, "Target")
        self.start_uwb_button = qt.QtWidgets.QPushButton("Start UWB")

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

        hLine4 = qt.QtWidgets.QFrame()
        hLine4.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine4.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        vSpacer  = qt.QtWidgets.QSpacerItem(20, 40, qt.QtWidgets.QSizePolicy.Minimum, qt.QtWidgets.QSizePolicy.Expanding)
       
        boxLayout.addLayout(self.uwb_left_layout)
        boxLayout.addWidget(hLine1)
        boxLayout.addLayout(self.uwb_right_layout)
        boxLayout.addWidget(hLine2)
        boxLayout.addLayout(self.odometry_layout)
        boxLayout.addWidget(hLine3)
        boxLayout.addLayout(self.odometry_values_layout)
        boxLayout.addItem(vSpacer)
        boxLayout.addWidget(self.start_uwb_button)

        group = qt.QtWidgets.QGroupBox(self.uav_name)
        group.setStyleSheet("QGroupBox { border: 1px solid black; }")
        # group.setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 1px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top; }")
        group.setLayout(boxLayout)

        self.layout.addWidget(group)

    def deinit(self):
        pass

    def start_uwb(self):
        if not (self.uwb_service == None):
            self.uwb_started = not self.uwb_started
            resp = self.uwb_service(self.uwb_started)
            if resp.success:
                if self.uwb_started:
                    # Clear Meters
                    self.uwb_left_layout.clear()
                    self.uwb_right_layout.clear()
                    self.odometry_layout.clear() 

                    self.start_uwb_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
                    self.start_uwb_button.setText("Kill UWB")
                else:
                    self.start_uwb_button.setStyleSheet("")
                    self.start_uwb_button.setText("Start UWB")
            else:
                self.start_uwb_button.setStyleSheet("QPushButton { background: red }")
                self.start_uwb_button.setText("UWB: Fail")
        else:
            self.start_uwb_button.setStyleSheet("QPushButton { background: red }")
            self.start_uwb_button.setText("UWB: Error")
            rospy.logwarn("Human Diagnostics : UWB Service Not Available")

    def start(self, cmd):
        if cmd:
            # Start UWB
            # Set States to Not-Started
            self.uwb_started = False
        else:
            # End UWB
            self.uwb_started = True

        # Start / End UWB
        self.start_uwb()

    def clear(self):
        self.uwb_left_layout.clear()
        self.uwb_right_layout.clear()
        self.odometry_layout.clear()
        
    
