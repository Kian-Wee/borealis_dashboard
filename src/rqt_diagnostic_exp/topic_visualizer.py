import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math

class TopicVisualize(QFormLayout):
    publish_rate_signal = qt.QtCore.pyqtSignal() 
    delay_signal = qt.QtCore.pyqtSignal(float)  #seconds
    max_delay_signal = qt.QtCore.pyqtSignal(float)   #seconds
    average_delay_signal = qt.QtCore.pyqtSignal()   
    start_record_signal = qt.QtCore.pyqtSignal(bool)

    def __init__(self, topic, msg_type, name=None):
        super(TopicVisualize, self).__init__()
        
        # Attributes
        self.topic = topic
        self.msg_type = msg_type
        if name is None:
            name = topic

        self.max_delay = 0.0
        self.total_delay = 0.0
        self.total_msg_count = 0

        self.createLayout(name)

        # Subscribers
        self.rate = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(self.topic, self.msg_type, self.rate.callback_hz, callback_args=self.topic)
        rospy.Subscriber(self.topic, self.msg_type, self.callback)

        # Signal Connections
        self.publish_rate_signal.connect(self.showMessageRate)
        self.delay_signal.connect(self.showMessageDelay)
        self.max_delay_signal.connect(self.showMaxDelay)
        self.average_delay_signal.connect(self.showAverageDelay)
        # self.value_out_of_range_signal.connect(self.showValueRange)
    
        # Start Class timer
        self.classTimer = self.ClassTimer(self.timerCallback)
        self.classTimer.start()


    def createLayout(self, name):
        rateDesc = qt.QtWidgets.QLabel(name)
        self.rate_label = qt.QtWidgets.QLabel("0")
        self.rate_label.setAlignment(qt.QtCore.Qt.AlignRight)

        delayDesc = qt.QtWidgets.QLabel("Delay(ms)")
        self.delay_label = qt.QtWidgets.QLabel("0")
        self.delay_label.setAlignment(qt.QtCore.Qt.AlignRight)

        maxDelayDesc = qt.QtWidgets.QLabel("Max Delay(ms)")
        self.max_delay_label = qt.QtWidgets.QLabel("0")
        self.max_delay_label.setAlignment(qt.QtCore.Qt.AlignRight)

        averageDelayDesc = qt.QtWidgets.QLabel("Avg Delay(ms)")
        self.average_delay_label = qt.QtWidgets.QLabel("0")
        self.average_delay_label.setAlignment(qt.QtCore.Qt.AlignRight)


        # valueRangeDesc = qt.QtWidgets.QLabel("Value Range")
        # self.value_range_label = qt.QtWidgets.QLabel("OK")
        # self.value_range_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
        # self.value_range_label.setAlignment(qt.QtCore.Qt.AlignCenter)

        self.addRow(rateDesc, self.rate_label)
        self.addRow(delayDesc, self.delay_label)
        self.addRow(maxDelayDesc, self.max_delay_label)
        self.addRow(averageDelayDesc, self.average_delay_label)

        # misc_layout.addRow(valueRangeDesc, self.value_range_label)

    def deinit(self):
        pass

    def clear(self):
        self.rate_label.setText('0')
        self.delay_label.setText('0')
        self.max_delay_label.setText('0')
        self.average_delay_label.setText('0')
        # self.value_range_label.setText('OK')

        self.max_delay = 0.0
        self.total_delay = 0.0
        self.total_msg_count = 0
        
        # Create new message rate counter
        self.rate = rostopic.ROSTopicHz(-1)
        rospy.Subscriber(self.topic, self.msg_type, self.rate.callback_hz, callback_args=self.topic)

        self.rate_label.setStyleSheet("")
        self.delay_label.setStyleSheet("")
        self.max_delay_label.setStyleSheet("")
        self.average_delay_label.setStyleSheet("")
        # self.value_range_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
        
    def callback(self, msg):
        # Calculate Delay
        msgTime = msg.header.stamp.secs +  msg.header.stamp.nsecs * 1e-9
        curentTime = rospy.get_time()
        msgDelay = curentTime - msgTime 

        # Update Max Delay
        if math.fabs(msgDelay) > math.fabs(self.max_delay):
            self.max_delay = msgDelay
            self.max_delay_signal.emit(self.max_delay)
        
        # for reading in msg.readings:
        #     if reading.distance > 10:
        #         self.value_out_of_range_signal.emit(False)
        #         break
        #     self.value_out_of_range_signal.emit(True)

        # Calculate Average
        self.total_delay += msgDelay
        self.total_msg_count += 1
        self.average_delay = float(self.total_delay) / self.total_msg_count

        self.delay_signal.emit(msgDelay)
        self.average_delay_signal.emit()
    
    def showMessageRate(self):
        try:
            rate = str(round( (self.rate.get_hz(topic=self.topic)[0]), 2))
            self.rate_label.setStyleSheet("")
        except:
            rate = 'None'
            self.rate_label.setStyleSheet("QLabel { background: red }")
        
        self.rate_label.setText(rate)

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


    # """ Show Value of range error
    #     :param status: Value out of range status
    #                     True : In Range
    #                     False : Out of range
    # """
    # def showValueRange(self, status):
    #     if status:
    #         self.value_range_label.setStyleSheet("QLabel { background: rgb(71, 255, 62) }")
    #         self.value_range_label.setText(str("OK"))
    #     else:
    #         self.value_range_label.setStyleSheet("QLabel { background: red }")
    #         self.value_range_label.setText(str("OOR"))
    
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

