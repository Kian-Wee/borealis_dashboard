import rospy, rostopic
import rospkg
import psutil

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from uwb_msgs.msg import UUBmsg, UWBReading
from std_srvs.srv import SetBool
from topic_visualizer import TopicVisualize
from local_position_visualizer import PositionVisualizer
from button_service import ButtonService

class Human_Diagnostic(QObject):
    footIMU_rate_signal = qt.QtCore.pyqtSignal() 
    Odometry_rate_signal = qt.QtCore.pyqtSignal()

    def __init__(self, layout, footIMU_topic, 
                    odometry_topic,
                    uwb_topic, 
                    odometry_service, 
                    fusion_service, 
                    glove_service, 
                    gun_service, 
                    hri_service, 
                    drone_yaw_control_service, 
                    uwb_service):
        super(Human_Diagnostic, self).__init__()

        # Attributes
        self.footIMU_topic = footIMU_topic
        self.odometry_topic = odometry_topic
        self.uwb_topic = uwb_topic
        self.odometry_started = False
        self.fusion_started = False

        # Create service buttons
        self.odometry_button = ButtonService(odometry_service, "Odometry", "Human")
        self.fusion_button = ButtonService(fusion_service, "Fusion", "Human")
        self.glove_button = ButtonService(glove_service, "Glove", "Human")
        self.gun_button = ButtonService(gun_service, "Rifle", "Human")
        self.HRI_button = ButtonService(hri_service, "HRI", "Human")
        self.drone_yaw_ctrl_button = ButtonService(drone_yaw_control_service, "Drone Yaw Ctrl", "Human")
        self.uwb_button = ButtonService(uwb_service, "UWB", "Human")

        self.layout = layout
        self.createLayout(self.layout)

        # Subscribers
        self.footIMU_rate = rostopic.ROSTopicHz(2)
        self.odometry_rate = rostopic.ROSTopicHz(2)
        rospy.Subscriber(self.footIMU_topic, Imu, self.footIMU_rate.callback_hz, callback_args=self.footIMU_topic)
        rospy.Subscriber(self.odometry_topic, Odometry, self.odometry_rate.callback_hz, callback_args=self.odometry_topic)

        # Signal Connections
        self.footIMU_rate_signal.connect(self.showFootImuMessageRate)
        self.Odometry_rate_signal.connect(self.showOdometryMessageRate)
        
        # Start Class timer
        self.classTimer = self.ClassTimer(self.timerCallback)
        self.classTimer.start()

        rospy.Rate(2) # Run at lower rate to prevent bandwidth saturation

    def createLayout(self, layout_):
        layout = qt.QtWidgets.QFormLayout()

        footIMUDesc = qt.QtWidgets.QLabel("Foot IMU Rate")
        self.footIMU_label = qt.QtWidgets.QLabel("0")
        self.footIMU_label.setAlignment(qt.QtCore.Qt.AlignRight)

        odometryDesc = qt.QtWidgets.QLabel("Odometry Rate")
        self.odometry_label = qt.QtWidgets.QLabel("0")
        self.odometry_label.setAlignment(qt.QtCore.Qt.AlignRight)

        layout.addRow(footIMUDesc, self.footIMU_label)
        layout.addRow(odometryDesc, self.odometry_label)

        boxLayout = qt.QtWidgets.QVBoxLayout()
        boxLayout.addSpacing(20)

        hLine1 = qt.QtWidgets.QFrame()
        hLine1.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine1.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        self.odom_viz = TopicVisualize(self.odometry_topic, Odometry, "Odom")

        hLine2 = qt.QtWidgets.QFrame()
        hLine2.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine2.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        self.uwb_viz = TopicVisualize(self.uwb_topic, UUBmsg, "UWB")

        hLine3 = qt.QtWidgets.QFrame()
        hLine3.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine3.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        self.odom = PositionVisualizer(self.odometry_topic, name="Odom")
       
        boxLayout.addLayout(layout)
        boxLayout.addWidget(hLine1)
        boxLayout.addLayout(self.uwb_viz)
        boxLayout.addWidget(hLine2)
        boxLayout.addLayout(self.odom_viz)
        boxLayout.addWidget(hLine3)
        boxLayout.addLayout(self.odom)

        # Add spacer
        vSpacer  = qt.QtWidgets.QSpacerItem(20, 40, qt.QtWidgets.QSizePolicy.Minimum, qt.QtWidgets.QSizePolicy.Expanding)
        boxLayout.addItem(vSpacer)

        boxLayout.addWidget(self.odometry_button)
        boxLayout.addWidget(self.uwb_button)
        boxLayout.addWidget(self.fusion_button)
        boxLayout.addWidget(self.glove_button)
        boxLayout.addWidget(self.gun_button)
        boxLayout.addWidget(self.HRI_button)
        boxLayout.addWidget(self.drone_yaw_ctrl_button)

        group = qt.QtWidgets.QGroupBox("Human")
        group.setStyleSheet("QGroupBox { border: 1px solid black; }")
        # group.setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 1px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top; }")
        group.setLayout(boxLayout)
        layout_.addWidget(group)

    def deinit(self):
        pass

    def clear(self):
        self.footIMU_label.setText('0')
        self.odometry_label.setText('0')
        self.footIMU_label.setStyleSheet("")
        self.odometry_label.setStyleSheet("")
        
    def showFootImuMessageRate(self):
        try:
            rate = str(round( (self.footIMU_rate.get_hz(topic=self.footIMU_topic)[0]), 2))
            self.footIMU_label.setStyleSheet("")
        except:
            rate = 'None'
            self.footIMU_label.setStyleSheet("QLabel { background: red }")
        
        self.footIMU_label.setText(rate)

    def showOdometryMessageRate(self):
        try:
            rate = str(round( (self.odometry_rate.get_hz(topic=self.odometry_topic)[0]), 2))
            self.odometry_label.setStyleSheet("")
        except:
            rate = 'None'
            self.odometry_label.setStyleSheet("QLabel { background: red }")
        
        self.odometry_label.setText(rate)

    def start(self, cmd):
        # Start / End Odometry
        self.odometry_button.call(cmd)
        # Start / End UWB
        self.uwb_button.call(cmd)
        # Start / End Fusion
        self.fusion_button.call(cmd)
        # Start / End Glove
        self.glove_button.call(cmd)
        # Start / End Rifle
        self.gun_button.call(cmd)
        # Start / End HRI
        self.HRI_button.call(cmd)
        # Start / End Drone Yaw Control
        self.drone_yaw_ctrl_button.call(cmd)

    """ 1 Hz Timer Callback
    """
    def timerCallback(self):
        # Publish Foot IMU Message Rate
        self.footIMU_rate_signal.emit()
        # Publish Odometry Message Rate
        self.Odometry_rate_signal.emit()

    class ClassTimer(QThread):
        def __init__(self, callback):
            QThread.__init__(self)
            self.timer = QTimer()
            # self.timer.moveToThread(self)
            self.timer.setInterval(int(1000))
            self.timer.timeout.connect(callback)
            self.timer.start()
            

        def __del__(self):
            self.wait()

        def run(self):        
            pass
