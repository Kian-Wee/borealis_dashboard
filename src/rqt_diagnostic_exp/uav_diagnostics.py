import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from uwb_msgs.msg import UUBmsg, UWBReading
from std_srvs.srv import SetBool
import math

class UAV_Diagnostic(QObject):
    publish_rate_signal = qt.QtCore.pyqtSignal() 
    delay_signal = qt.QtCore.pyqtSignal(float)  #seconds
    max_delay_signal = qt.QtCore.pyqtSignal(float)   #seconds
    average_delay_signal = qt.QtCore.pyqtSignal()   
    delay_std_deviation_signal = qt.QtCore.pyqtSignal()
    port_status_signal = qt.QtCore.pyqtSignal(str)
    ping_status_signal = qt.QtCore.pyqtSignal(str)
    start_record_signal = qt.QtCore.pyqtSignal(bool)
    value_out_of_range_signal = qt.QtCore.pyqtSignal(bool)

    def __init__(self, widget, topic, uav_name):
        super(UAV_Diagnostic, self).__init__()
        
        # Attributes
        self.topic = topic
        self.widget = widget
        self.max_delay = 0.0
        self.average_delay = 0.0
        self.delay_std_deviation = 0.0
        self.total_delay = 0.0
        self.total_squared_delay = 0.0
        self.total_msg_count = 0
        self.record = False
        self.uav_name = uav_name

        # Retrieve GUI Components
        self.publish_rate_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_publish_rate_label')
        self.delay_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_delay_label')
        self.max_delay_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_max_delay_label')
        self.average_delay_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_average_delay_label')
        self.delay_std_deviation_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_delay_std_deviation_label')
        self.port_status_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_port_status_label')
        self.ping_status_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_ping_status_label')
        self.value_range_label = self.widget.findChild(qt.QtWidgets.QLabel, uav_name + '_value_range_label')
        self.record_button = self.widget.findChild(qt.QtWidgets.QPushButton, uav_name + '_record_pushButton')
        

        # Subscribers
        self.rate = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(topic, UUBmsg, self.rate.callback_hz, callback_args=topic)
        rospy.Subscriber(topic, UUBmsg, self.UUBCallback)
        # TODO Subscribe to Diagnostic Topic

        # Services
        service_name = uav_name + '_bag_recorder/record_bag'
        self.record_service = None
        try:
            rospy.wait_for_service(service_name, timeout=3)
            self.record_service = rospy.ServiceProxy(service_name, SetBool)
        except Exception as e:
            self.record_button.setStyleSheet("QPushButton { background: red }")
            self.record_button.setText("Error")
            rospy.logwarn ("UAV Diagnostics :" + self.uav_name + " Exception:" + str(e))

        # Signal Connections
        self.publish_rate_signal.connect(self.showMessageRate)
        self.delay_signal.connect(self.showMessageDelay)
        self.max_delay_signal.connect(self.showMaxDelay)
        self.average_delay_signal.connect(self.showAverageDelay)
        self.delay_std_deviation_signal.connect(self.showDelayStdDeviation)
        self.value_out_of_range_signal.connect(self.showValueRange)
        self.record_button.clicked.connect(self.recordBag)
        

        # Start Class timer
        self.classTimer = self.ClassTimer(self.timerCallback)
        self.classTimer.start()

    def deinit(self):
        pass

    def clear(self):
        self.publish_rate_label.setText('0')
        self.delay_label.setText('0')
        self.max_delay_label.setText('0')
        self.average_delay_label.setText('0')
        self.delay_std_deviation_label.setText('0')
        self.port_status_label.setText('OK')
        self.ping_status_label.setText('OK')
        self.value_range_label.setText('OK')
        self.record_button.setText('Start Recording')
        self.max_delay = 0.0
        self.total_delay = 0.0
        self.total_squared_delay = 0.0
        self.total_msg_count = 0

        self.publish_rate_label.setStyleSheet("")
        self.delay_label.setStyleSheet("")
        self.max_delay_label.setStyleSheet("")
        self.average_delay_label.setStyleSheet("")
        self.delay_std_deviation_label.setStyleSheet("")
        self.port_status_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
        self.ping_status_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
        self.value_range_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
        self.record_button.setStyleSheet("")
        
    def UUBCallback(self, msg):
        # Calculate Delay
        msgTime = msg.header.stamp.secs +  msg.header.stamp.nsecs * 1e-9
        curentTime = rospy.get_time()
        msgDelay = curentTime - msgTime 

        # Update Max Delay
        if math.fabs(msgDelay) > math.fabs(self.max_delay):
            self.max_delay = msgDelay
            self.max_delay_signal.emit(self.max_delay)
        
        for reading in msg.readings:
            if reading.distance > 10:
                self.value_out_of_range_signal.emit(False)
                break
            self.value_out_of_range_signal.emit(True)

        # Calculate Average & Std Deviation
        self.total_delay += msgDelay
        self.total_squared_delay += (msgDelay**2)
        self.total_msg_count += 1
        self.average_delay = float(self.total_delay) / self.total_msg_count
        self.delay_std_deviation = math.sqrt(  float(self.total_squared_delay) / self.total_msg_count - math.pow(self.average_delay, 2) )

        self.delay_signal.emit(msgDelay)
        self.average_delay_signal.emit()
        self.delay_std_deviation_signal.emit()
    
    def showMessageRate(self):
        try:
            rate = str(round( (self.rate.get_hz(topic=self.topic)[0]), 2))
            self.publish_rate_label.setStyleSheet("")
        except:
            rate = 'None'
            self.publish_rate_label.setStyleSheet("QLabel { background: red }")
        
        self.publish_rate_label.setText(rate)

    def showMessageDelay(self, delay):
        delay = round( delay * 1000, 2) # Milliseconds
        if abs(delay) > 100:
            self.delay_label.setStyleSheet("QLabel { background: red }")
        else:
            self.delay_label.setStyleSheet("")

        self.delay_label.setText(str(delay))

    def showMaxDelay(self, maxDelay):
        maxDelay = round( maxDelay * 1000, 2) # Milliseconds
        if abs(maxDelay) > 100:
            self.max_delay_label.setStyleSheet("QLabel { background: rgb(255, 170, 0) }")
        else:
            self.max_delay_label.setStyleSheet("")

        self.max_delay_label.setText(str(maxDelay))

    def showAverageDelay(self):
        averageDelay = round( self.average_delay * 1000, 2) # Milliseconds
        if abs(averageDelay) > 100:
            self.average_delay_label.setStyleSheet("QLabel { background: rgb(255, 170, 0) }")
        else:
            self.average_delay_label.setStyleSheet("")

        self.average_delay_label.setText(str(averageDelay))

    def showDelayStdDeviation(self):
        delayStdDeviation = round( self.delay_std_deviation * 1000, 2) # Milliseconds
        if abs(delayStdDeviation) > 100:
            self.delay_std_deviation_label.setStyleSheet("QLabel { background: rgb(255, 170, 0) }")
        else:
            self.delay_std_deviation_label.setStyleSheet("")

        self.delay_std_deviation_label.setText(str(delayStdDeviation))

    """ Show Value of range error
        :param status: Value out of range status
                        True : In Range
                        False : Out of range
    """
    def showValueRange(self, status):
        if status:
            self.value_range_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
            self.value_range_label.setText(str("OK"))
        else:
            self.value_range_label.setStyleSheet("QLabel { background: red }")
            self.value_range_label.setText(str("OOR"))

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
            self.record_button.setStyleSheet("QPushButton { background: red }")
            self.record_button.setText("Error")
            rospy.logwarn("UAV Diagnostics : " + self.uav_name + " Record Service Not Available")
    
    """ 1 Hz Timer Callback
    """
    def timerCallback(self):
        # Publish Message Rate
        self.publish_rate_signal.emit()
        
        

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
