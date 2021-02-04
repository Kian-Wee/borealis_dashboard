import rospy, rostopic
import rospkg
import psutil

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from topic_visualizer import TopicVisualize

class Human_Diagnostic(QObject):
    footIMU_rate_signal = qt.QtCore.pyqtSignal() 
    Odometry_rate_signal = qt.QtCore.pyqtSignal()

    def __init__(self, layout, footIMU_topic, odometry_topic, odometry_service, fusion_service):
        super(Human_Diagnostic, self).__init__()
        
        # Attributes
        self.footIMU_topic = footIMU_topic
        self.odometry_topic = odometry_topic
        self.odometry_started = False
        self.fusion_started = False

        self.layout = layout
        self.createLayout(self.layout)


        # Subscribers
        self.footIMU_rate = rostopic.ROSTopicHz(1000)
        self.odometry_rate = rostopic.ROSTopicHz(1000)
        rospy.Subscriber(self.footIMU_topic, Imu, self.footIMU_rate.callback_hz, callback_args=self.footIMU_topic)
        rospy.Subscriber(self.odometry_topic, Odometry, self.odometry_rate.callback_hz, callback_args=self.odometry_topic)

        # Services
        self.odometry_service = None
        try:
            rospy.wait_for_service(odometry_service, timeout=3)
            self.odometry_service = rospy.ServiceProxy(odometry_service, SetBool)
        except Exception as e:
            self.start_odometry_button.setStyleSheet("QPushButton { background: red }")
            self.start_odometry_button.setText("Odometry: Error")
            rospy.logwarn ("Human Diagnostics : Exception:" + str(e))
        
        self.fusion_service = None
        try:
            rospy.wait_for_service(fusion_service, timeout=3)
            self.fusion_service = rospy.ServiceProxy(fusion_service, SetBool)
        except Exception as e:
            self.start_fusion_button.setStyleSheet("QPushButton { background: red }")
            self.start_fusion_button.setText("Fusion: Error")
            rospy.logwarn ("Human Diagnostics : Exception:" + str(e))

        # Signal Connections
        self.footIMU_rate_signal.connect(self.showFootImuMessageRate)
        self.Odometry_rate_signal.connect(self.showOdometryMessageRate)
        self.start_odometry_button.clicked.connect(self.start_odometry)
        self.start_fusion_button.clicked.connect(self.start_fusion)
        
        # Start Class timer
        self.classTimer = self.ClassTimer(self.timerCallback)
        self.classTimer.start()

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

        self.start_odometry_button = qt.QtWidgets.QPushButton("Start Odometry")
        self.start_fusion_button = qt.QtWidgets.QPushButton("Start Fusion")

        boxLayout = qt.QtWidgets.QVBoxLayout()
        boxLayout.addSpacing(20)

        hLine1 = qt.QtWidgets.QFrame()
        hLine1.setFrameShape(qt.QtWidgets.QFrame.HLine)
        hLine1.setFrameShadow(qt.QtWidgets.QFrame.Sunken)

        self.odom_viz = TopicVisualize(self.odometry_topic, Odometry, "Odom")
       
        boxLayout.addLayout(layout)
        boxLayout.addWidget(hLine1)
        boxLayout.addLayout(self.odom_viz)

        # Add spacer
        vSpacer  = qt.QtWidgets.QSpacerItem(20, 40, qt.QtWidgets.QSizePolicy.Minimum, qt.QtWidgets.QSizePolicy.Expanding)
        boxLayout.addItem(vSpacer)

        boxLayout.addWidget(self.start_odometry_button)
        boxLayout.addWidget(self.start_fusion_button)

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

    def start_odometry(self):
        if not (self.odometry_service == None):
            self.odometry_started = not self.odometry_started
            resp = self.odometry_service(self.odometry_started)
            if resp.success:
                if self.odometry_started:
                    # Clear meters
                    self.odom_viz.clear()
                    
                    self.start_odometry_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
                    self.start_odometry_button.setText("Kill Odometry")
                else:
                    self.start_odometry_button.setStyleSheet("")
                    self.start_odometry_button.setText("Start Odometry")
            else:
                self.start_odometry_button.setStyleSheet("QPushButton { background: red }")
                self.start_odometry_button.setText("Odometry: Fail")
        else:
            self.start_odometry_button.setStyleSheet("QPushButton { background: red }")
            self.start_odometry_button.setText("Odometry: Error")
            rospy.logwarn("Human Diagnostics : Odometry Service Not Available")

    def start_fusion(self):
        if not (self.fusion_service == None):
            self.fusion_started = not self.fusion_started
            resp = self.fusion_service(self.fusion_started)
            if resp.success:
                if self.fusion_started:
                    self.start_fusion_button.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
                    self.start_fusion_button.setText("Kill Fusion")
                else:
                    self.start_fusion_button.setStyleSheet("")
                    self.start_fusion_button.setText("Start Fusion")
            else:
                self.start_fusion_button.setStyleSheet("QPushButton { background: red }")
                self.start_fusion_button.setText("Fusion: Fail")
        else:
            self.start_fusion_button.setStyleSheet("QPushButton { background: red }")
            self.start_fusion_button.setText("Fusion: Error")
            rospy.logwarn("Human Diagnostics : Fusion Service Not Available")

    def start(self, cmd):
        if cmd:
            # Start Odometry and Fusion
            # Set States to Not-Started
            self.odometry_started = False
            self.fusion_started = False
        else:
            # End Odometry and Fusion
            self.odometry_started = True
            self.fusion_started = True

        # Start / End Odometry
        self.start_odometry()
        # Start / End Fusion
        self.start_fusion()

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
