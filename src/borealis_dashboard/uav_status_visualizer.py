import rospy, rostopic
import rospkg

import PyQt5 as qt
from PyQt5.QtCore import QObject, QThread, QTimer
from PyQt5.QtWidgets import QFormLayout
import math
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from mt_msgs.msg import phaseAndTime
from pythonping import ping 
import re

class UAVStatusVisualizer(QFormLayout):
    flightmode_signal = qt.QtCore.pyqtSignal(str) #flight mode
    battery_signal = qt.QtCore.pyqtSignal(float) #Battery level
    mode_signal = qt.QtCore.pyqtSignal(str)
    planner_signal = qt.QtCore.pyqtSignal(str)

    teaming_planner_rf_signal = qt.QtCore.pyqtSignal(str) # phaseAndTime.phase is actually u_int8 but whatever 
    teaming_planner_cp_signal = qt.QtCore.pyqtSignal(str)


    def __init__(self, state_topic, battery_topic, mode_topic, planner_topic, teaming_planner_rf_phasesync_topic, teaming_planner_cp_status_topic, ping_server, name="UAV Status"):
        super(UAVStatusVisualizer, self).__init__()
        
        # Attributes
        self.state_topic = state_topic
        self.battery_topic = battery_topic
        self.mode_topic = mode_topic
        self.planner_topic = planner_topic

        self.teaming_planner_rf_phasesync_topic = teaming_planner_rf_phasesync_topic # Robot Formation Phases
        self.teaming_planner_cp_phasesync_topic = teaming_planner_cp_status_topic # Consensus Global path Phases
        self.ping_server = ping_server # server to ping to, kian wee lapotop '192.168.1.3'

        # Start Ping Timer thread, ping might be a blocking function so a seperate thread is used
        self.ping_timer_thread = self.PingTimerThread(self.ping_timer_callback)
        self.ping_timer_thread.start()

        self.createLayout(name)

        # Subscribers
        rospy.Subscriber(self.state_topic, State, self.state_callback)
        rospy.Subscriber(self.battery_topic, BatteryState, self.battery_callback)
        rospy.Subscriber(self.mode_topic, String, self.mode_callback)
        rospy.Subscriber(self.planner_topic, String, self.planner_callback)

        rospy.Subscriber(self.teaming_planner_rf_phasesync_topic, phaseAndTime, self.phasesync_rf_callback)
        rospy.Subscriber(self.teaming_planner_cp_phasesync_topic, phaseAndTime, self.phasesync_cp_callback)

        self.flightmode_signal.connect(self.showflightmodeStatus)
        self.battery_signal.connect(self.showbatteryStatus)
        self.mode_signal.connect(self.showmodeStatus)
        self.planner_signal.connect(self.showplannerStatus)

        rospy.Rate(2) # Run at lower rate to prevent bandwidth saturation


    def createLayout(self, name):
        WidgetDesc = qt.QtWidgets.QLabel(name)
        BatteryDesc = qt.QtWidgets.QLabel("    Battery Level:")
        FlightModeDesc = qt.QtWidgets.QLabel("    Flight Mode:")
        ModeDesc = qt.QtWidgets.QLabel("    Mode:")
        PlannerDesc = qt.QtWidgets.QLabel("    Planner:")

        PlannerRFDesc = qt.QtWidgets.QLabel("    Robot Formation Phase:")
        PlannerCPDesc = qt.QtWidgets.QLabel("    Consensus Path Phase:")
        PingDesc = qt.QtWidgets.QLabel("    Ping:")

        self.Battery_label = qt.QtWidgets.QLabel("")
        self.FlightMode_label = qt.QtWidgets.QLabel("")
        self.Mode_label = qt.QtWidgets.QLabel("")
        self.Planner_label = qt.QtWidgets.QLabel("")

        self.PlannerRFDesc_label = qt.QtWidgets.QLabel("")
        self.PlannerCPDesc_label = qt.QtWidgets.QLabel("")
        self.PingDesc_label = qt.QtWidgets.QLabel("")

        self.addRow(WidgetDesc, qt.QtWidgets.QLabel(""))
        self.addRow(BatteryDesc, self.Battery_label)
        self.addRow(FlightModeDesc, self.FlightMode_label)
        self.addRow(ModeDesc, self.Mode_label)
        self.addRow(PlannerDesc, self.Planner_label)

        self.addRow(PlannerRFDesc, self.PlannerRFDesc_label)
        self.addRow(PlannerCPDesc, self.PlannerCPDesc_label)
        self.addRow(PingDesc, self.PingDesc_label)


    def deinit(self):
        pass

    def clear(self):
        self.Battery_label.setText("")
        self.FlightMode_label.setText("")
        self.Mode_label.setText("")
        
    def state_callback(self, msg):
        self.flightmode_signal.emit(msg.mode)

    def battery_callback(self, msg):
        batterymin=3.2
        batterymax=4.2
        numcells=6 # number of cells in drone
        x = msg.cell_voltage # Array of battery voltages
        if len(x)==0:
            self.battery_signal.emit(0)
        elif len(x)==1: # in newer versions of px4, instead of giving a list of single cell voltages it gives the raw voltage
            battery=x[0]
            batterylevel= (battery-batterymin*numcells)/(batterymax-batterymin)/numcells
            self.battery_signal.emit(batterylevel)
        else:
            battery=sum(x)/len(x)
            batterylevel= (battery-batterymin)/(batterymax-batterymin)
            self.battery_signal.emit(batterylevel)

    def mode_callback(self, msg):
        self.mode_signal.emit(msg.data)

    def planner_callback(self, msg):
        self.planner_signal.emit(msg.data)
    
    def phasesync_rf_callback(self, msg):
        self.teaming_planner_rf_signal.emit(str(msg.data.phase))

    def phasesync_cp_callback(self, msg):
        self.teaming_planner_cp_signal.emit(str(msg.data.phase))

    def showflightmodeStatus(self, data):
        self.FlightMode_label.setText(data)

    def showbatteryStatus(self,x):
        self.Battery_label.setText("%.2f"%x)

    def showmodeStatus(self, data):
        self.Mode_label.setText(data)

    def showplannerStatus(self, data):
        self.Planner_label.setText(data)

    def showTeamingPlannerRFStatus(self, data):
        self.PlannerRFDesc_label.setText(data)

    def showTeamingPlannerCPStatus(self, data):
        self.PlannerCPDesc_label.setText(data)

    def ping_timer_callback(self):
        out = ping(self.ping_server, verbose=False, count=1, size=36, timeout=6)

        time_out_flag = "Request timed out"
        if re.findall(str(out), time_out_flag): # if output has the sentence Reques timed out ..
            print("Ping time out!")
            self.PingDesc_label.setText("Time out! >6s")
        else:
            out = str(out).split('/') # the avg substring in the output string
            avg = out[3]
            print("Ping is {}".format(str(avg)))
            self.PingDesc_label.setText(str(avg))
        pass

class PingTimerThread(QThread):
    def __init__(self, callback):
        QThread.__init__(self)
        self.timer = QTimer()
        self.timer.setInterval(int(1000)) # in ms
        self.timer.timeout.connect(callback)
        self.timer.start()
        
    def __del__(self):
        self.wait()

    def run(self):        
        pass

