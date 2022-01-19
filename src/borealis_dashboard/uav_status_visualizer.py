import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

class UAVStatusVisualizer(QFormLayout):
    flightmode_signal = qt.QtCore.pyqtSignal(str) #flight mode
    battery_signal = qt.QtCore.pyqtSignal(float) #Battery level

    def __init__(self, state_topic, battery_topic, name="UAV Status"):
        super(UAVStatusVisualizer, self).__init__()
        
        # Attributes
        self.state_topic = state_topic
        self.battery_topic = battery_topic

        self.createLayout(name)

        # Subscribers
        rospy.Subscriber(self.state_topic, State, self.state_callback)
        rospy.Subscriber(self.battery_topic, BatteryState, self.battery_callback)
        
        self.flightmode_signal.connect(self.showmodeStatus)
        self.battery_signal.connect(self.showbatteryStatus)


    def createLayout(self, name):
        WidgetDesc = qt.QtWidgets.QLabel(name)
        BatteryDesc = qt.QtWidgets.QLabel("    Battery Level:")
        FlightModeDesc = qt.QtWidgets.QLabel("    Flight Mode:") # Need to write node to include other flight modes eg follow me
        self.odomX_label = qt.QtWidgets.QLabel("")
        self.odomY_label = qt.QtWidgets.QLabel("")

        self.addRow(WidgetDesc, qt.QtWidgets.QLabel(""))
        self.addRow(BatteryDesc, self.odomX_label)
        self.addRow(FlightModeDesc, self.odomY_label)

    def deinit(self):
        pass

    def clear(self):
        self.odomX_label.setText("")
        self.odomY_label.setText("")
        
    def state_callback(self, msg):
        y = msg.mode
        self.flightmode_signal.emit(y)

    def battery_callback(self, msg):
        x = msg.cell_voltage # Array of battery voltages
        if len(x)==0:
            self.battery_signal.emit(x)
        else:
            battery=sum(x)/len(x)
            batterymin=3.2
            batterymax=4.2
            batterylevel= (battery-batterymin)/(batterymax-batterymin)
            x=batterylevel
            self.battery_signal.emit(x)

    
    def showmodeStatus(self, y):
        self.odomY_label.setText(y) #TEST

    def showbatteryStatus(self,x):
        self.odomX_label.setText("%.2f"%x)