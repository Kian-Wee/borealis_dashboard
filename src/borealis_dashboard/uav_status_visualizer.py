import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

class UAVStatusVisualizer(QFormLayout):
    flightmode_signal = qt.QtCore.pyqtSignal(str) #flight mode
    battery_signal = qt.QtCore.pyqtSignal(float) #Battery level
    mode_signal = qt.QtCore.pyqtSignal(float) #Battery level

    def __init__(self, state_topic, battery_topic, mode_topic, name="UAV Status"):
        super(UAVStatusVisualizer, self).__init__()
        
        # Attributes
        self.state_topic = state_topic
        self.battery_topic = battery_topic
        self.mode_topic = mode_topic

        self.createLayout(name)

        # Subscribers
        rospy.Subscriber(self.state_topic, State, self.state_callback)
        rospy.Subscriber(self.battery_topic, BatteryState, self.battery_callback)
        rospy.Subscriber(self.mode_topic, String, self.mode_callback)
        
        self.flightmode_signal.connect(self.showflightmodeStatus)
        self.battery_signal.connect(self.showbatteryStatus)
        self.mode_signal.connect(self.showmodeStatus)


    def createLayout(self, name):
        WidgetDesc = qt.QtWidgets.QLabel(name)
        BatteryDesc = qt.QtWidgets.QLabel("    Battery Level:")
        FlightModeDesc = qt.QtWidgets.QLabel("    Flight Mode:")
        ModeDesc = qt.QtWidgets.QLabel("    Mode:")
        self.Battery_label = qt.QtWidgets.QLabel("")
        self.FlightMode_label = qt.QtWidgets.QLabel("")
        self.Mode_label = qt.QtWidgets.QLabel("")

        self.addRow(WidgetDesc, qt.QtWidgets.QLabel(""))
        self.addRow(BatteryDesc, self.Battery_label)
        self.addRow(FlightModeDesc, self.FlightMode_label)
        self.addRow(ModeDesc, self.Mode_label)

    def deinit(self):
        pass

    def clear(self):
        self.Battery_label.setText("")
        self.FlightMode_label.setText("")
        self.Mode_label.setText("")
        
    def state_callback(self, msg):
        self.flightmode_signal.emit(msg.mode)

    def battery_callback(self, msg):
        x = msg.cell_voltage # Array of battery voltages
        if len(x)==0:
            self.battery_signal.emit(0)
        else:
            battery=sum(x)/len(x)
            batterymin=3.2
            batterymax=4.2
            batterylevel= (battery-batterymin)/(batterymax-batterymin)
            x=batterylevel
            self.battery_signal.emit(x)

    def mode_callback(self, msg):
        self.mode_signal.emit(msg.data)
    
    def showflightmodeStatus(self, data):
        self.FlightMode_label.setText(data)

    def showbatteryStatus(self,x):
        self.Battery_label.setText("%.2f"%x)

    def showmodeStatus(self, data):
        self.Mode_label.setText(data)