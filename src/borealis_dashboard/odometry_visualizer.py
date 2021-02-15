import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from nav_msgs.msg import Odometry

class OdometryVisualizer(QFormLayout):
    odometry_signal = qt.QtCore.pyqtSignal(float, float, float) # x, y

    def __init__(self, topic, name=None):
        super(OdometryVisualizer, self).__init__()
        
        # Attributes
        self.topic = topic
        if name is None:
            name = topic

        self.createLayout(name)

        # Subscribers
        rospy.Subscriber(self.topic, Odometry, self.callback)
        
        self.odometry_signal.connect(self.showOdometry)


    def createLayout(self, name):
        odomDesc = qt.QtWidgets.QLabel(name)
        odomXDesc = qt.QtWidgets.QLabel("    x         :")
        odomYDesc = qt.QtWidgets.QLabel("    y         :")
        odomThetaDesc = qt.QtWidgets.QLabel("    Theta:")
        self.odomX_label = qt.QtWidgets.QLabel("")
        self.odomY_label = qt.QtWidgets.QLabel("")
        self.odomTheta_label = qt.QtWidgets.QLabel("")

        self.addRow(odomDesc, qt.QtWidgets.QLabel("[m, m, deg]"))
        self.addRow(odomXDesc, self.odomX_label)
        self.addRow(odomYDesc, self.odomY_label)
        self.addRow(odomThetaDesc, self.odomTheta_label)

    def deinit(self):
        pass

    def clear(self):
        self.odomX_label.setText("")
        self.odomY_label.setText("")
        self.odomTheta_label.setText("")
        
    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z * 180 / math.pi
        self.odometry_signal.emit(x, y, theta)
    
    def showOdometry(self, x, y, theta):
        self.odomX_label.setText("%.2f"%x)
        self.odomY_label.setText("%.2f"%y)
        self.odomTheta_label.setText("%.2f"%theta)