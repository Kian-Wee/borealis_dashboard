import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from nav_msgs.msg import Odometry
import tf

class PositionVisualizer(QFormLayout):
    odometry_signal = qt.QtCore.pyqtSignal(float, float, float, float) # x, y, z, theta

    def __init__(self, topic, name=None):
        super(PositionVisualizer, self).__init__()
        
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
        odomZDesc = qt.QtWidgets.QLabel("    z         :")
        odomThetaDesc = qt.QtWidgets.QLabel("    Deg:")
        self.odomX_label = qt.QtWidgets.QLabel("")
        self.odomY_label = qt.QtWidgets.QLabel("")
        self.odomZ_label = qt.QtWidgets.QLabel("")
        self.odomTheta_label = qt.QtWidgets.QLabel("")

        self.addRow(odomDesc, qt.QtWidgets.QLabel("[m, m, m, deg]"))
        self.addRow(odomXDesc, self.odomX_label)
        self.addRow(odomYDesc, self.odomY_label)
        self.addRow(odomZDesc, self.odomZ_label)
        self.addRow(odomThetaDesc, self.odomTheta_label)

    def deinit(self):
        pass

    def clear(self):
        self.odomX_label.setText("")
        self.odomY_label.setText("")
        self.odomZ_label.setText("")
        self.odomTheta_label.setText("")
        
    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(euler[2])
        self.odometry_signal.emit(x, y, z, yaw)
    
    def showOdometry(self, x, y, z, theta):
        self.odomX_label.setText("%.2f"%x)
        self.odomY_label.setText("%.2f"%y)
        self.odomZ_label.setText("%.2f"%z)
        self.odomTheta_label.setText("%.2f"%theta)
