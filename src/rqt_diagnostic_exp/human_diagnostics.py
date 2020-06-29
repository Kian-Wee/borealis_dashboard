import rospy, rostopic
import rospkg
import psutil

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

class Human_Diagnostic(QObject):
    footIMU_rate_signal = qt.QtCore.pyqtSignal() 
    waistIMU_rate_signal = qt.QtCore.pyqtSignal() 
    Odometry_rate_signal = qt.QtCore.pyqtSignal() 
    cpu_usage_signal = qt.QtCore.pyqtSignal() 
    ram_usage_signal = qt.QtCore.pyqtSignal()
    

    def __init__(self, widget, footIMU_topic, waistIMU_topic, odometry_topic):
        super(Human_Diagnostic, self).__init__()
        
        # Attributes
        self.footIMU_topic = footIMU_topic
        self.waistIMU_topic = waistIMU_topic
        self.odometry_topic = odometry_topic
        self.record = False

        self.widget = widget

        # Retrieve GUI Components
        self.footIMU_label = self.widget.findChild(qt.QtWidgets.QLabel,  'human_footIMU_rate_label')
        self.waistIMU_label = self.widget.findChild(qt.QtWidgets.QLabel, 'human_waistIMU_rate_label')
        self.odometry_label = self.widget.findChild(qt.QtWidgets.QLabel, 'human_odometry_rate_label')
        self.cpu_usage_label = self.widget.findChild(qt.QtWidgets.QLabel, 'human_CPU_usage_label')
        self.ram_usage_label = self.widget.findChild(qt.QtWidgets.QLabel, 'human_RAM_usage_label')
        self.record_button = self.widget.findChild(qt.QtWidgets.QPushButton, 'human_record_pushButton')

        # Subscribers
        self.footIMU_rate = rostopic.ROSTopicHz(-1)
        self.waistIMU_rate = rostopic.ROSTopicHz(-1)
        self.odometry_rate = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(self.footIMU_topic, Imu, self.footIMU_rate.callback_hz, callback_args=self.footIMU_topic)
        rospy.Subscriber(self.waistIMU_topic, Imu, self.waistIMU_rate.callback_hz, callback_args=self.waistIMU_topic)
        rospy.Subscriber(self.odometry_topic, Odometry, self.odometry_rate.callback_hz, callback_args=self.odometry_topic)

        # Services
        service_name = 'human_bag_recorder/record_bag'
        self.record_service = None
        try:
            rospy.wait_for_service(service_name, timeout=3)
            self.record_service = rospy.ServiceProxy(service_name, SetBool)
        except:
            self.record_button.setStyleSheet("QPushButton { background: red }")
            self.record_button.setText("Error")
            rospy.logerr("Human Diagnostics: " + service_name + " Not available")

        # Signal Connections
        self.footIMU_rate_signal.connect(self.showFootImuMessageRate)
        self.waistIMU_rate_signal.connect(self.showWaistImuMessageRate)
        self.Odometry_rate_signal.connect(self.showOdometryMessageRate)
        self.cpu_usage_signal.connect(self.showCpuUsage)
        self.ram_usage_signal.connect(self.showRamUsage)
        self.record_button.clicked.connect(self.recordBag)

        
        # Start Class timer
        self.classTimer = self.ClassTimer(self.timerCallback)
        self.classTimer.start()

    def deinit(self):
        pass

    def clear(self):
        self.footIMU_label.setText('0')
        self.waistIMU_label.setText('0')
        self.odometry_label.setText('0')
        self.cpu_usage_label.setText('0%')
        self.ram_usage_label.setText('0%')
        self.record_button.setText('Start Recording')

        self.footIMU_label.setStyleSheet("")
        self.waistIMU_label.setStyleSheet("")
        self.odometry_label.setStyleSheet("")
        self.cpu_usage_label.setStyleSheet("")
        self.ram_usage_label.setStyleSheet("")
        self.record_button.setStyleSheet("")
        
    def showFootImuMessageRate(self):
        try:
            rate = str(round( (self.footIMU_rate.get_hz(topic=self.footIMU_topic)[0]), 2))
            self.footIMU_label.setStyleSheet("")
        except:
            rate = 'None'
            self.footIMU_label.setStyleSheet("QLabel { background: red }")
        
        self.footIMU_label.setText(rate)

    def showWaistImuMessageRate(self):
        try:
            rate = str(round( (self.waistIMU_rate.get_hz(topic=self.waistIMU_topic)[0]), 2))
            self.waistIMU_label.setStyleSheet("")
        except:
            rate = 'None'
            self.waistIMU_label.setStyleSheet("QLabel { background: red }")
        
        self.waistIMU_label.setText(rate)

    def showOdometryMessageRate(self):
        try:
            rate = str(round( (self.odometry_rate.get_hz(topic=self.odometry_topic)[0]), 2))
            self.odometry_label.setStyleSheet("")
        except:
            rate = 'None'
            self.odometry_label.setStyleSheet("QLabel { background: red }")
        
        self.odometry_label.setText(rate)

    def showCpuUsage(self):
        cpu_usage = psutil.cpu_percent()
        if cpu_usage > 90:
            self.cpu_usage_label.setStyleSheet("QLabel { background: red }")
        else:
            self.cpu_usage_label.setStyleSheet("")

        self.cpu_usage_label.setText(str(round(cpu_usage)) + '%')

    def showRamUsage(self):
        ram_usage = psutil.virtual_memory().percent
        if ram_usage > 90:
            self.ram_usage_label.setStyleSheet("QLabel { background: red }")
        else:
            self.ram_usage_label.setStyleSheet("")
        self.ram_usage_label.setText(str(round(ram_usage)) + '%')

    def recordBag(self):
        if not (self.record_service == None):
            self.record = not self.record
            resp = self.record_service(self.record)
            if resp.success:
                if self.record:
                    self.record_button.setStyleSheet("QPushButton { background: qlineargradient(spread:pad, x1:1, y1:1, x2:1, y2:0, stop:0 rgba(255, 0, 0, 215), stop:1 rgba(255, 255, 255, 255)) }")
                    self.record_button.setText("Stop Recording")
                else:
                    self.record_button.setStyleSheet("")
                    self.record_button.setText("Stop Recording")
            else:
                self.record_button.setStyleSheet("QPushButton { background: red }")
        else:
            rospy.logwarn("Human Diagnostics : Record Service Not Available")

    """ 1 Hz Timer Callback
    """
    def timerCallback(self):
        # Publish Foot IMU Message Rate
        self.footIMU_rate_signal.emit()
        # Publish Waist IMU Message Rate
        self.waistIMU_rate_signal.emit()
        # Publish Odometry Message Rate
        self.Odometry_rate_signal.emit()
        # Publish CPU Usage
        self.cpu_usage_signal.emit()
        # Publish RAM Usage
        self.ram_usage_signal.emit()

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