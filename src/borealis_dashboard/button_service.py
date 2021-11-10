import rospy
from std_srvs.srv import SetBool

import PyQt5 as qt
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QThread, QTimer

class ButtonService(QPushButton):

    def __init__(self, service_name, button_name, module_name="", callback=None):
        """
            Qt Push button to call a given service upon a click
            @param service_name: Service name to be called
            @param button_name: Button description
            @param module_name: Module name of the button parent (Error message printing only)
            @callback: Callback method which takes a bool input returning button state
        """
        super(ButtonService, self).__init__()

        self.service_name = service_name
        self.button_name = button_name
        self.module_name = module_name
        self.callback = callback

        # Set button text
        self.setText("Start " + self.button_name)
        # Init state
        self.enabled = False
        
        # Start connection handler thread
        self.service_connection = self.ServiceConnection(self.service_name, self.service_error_cb, self.service_connected_cb)

        self.clicked.connect(self.button_press)
    
    def __del__(self):
        self.connectionHandler.stop_connection()

    def button_press(self):
        if self.enabled:
            # Send OFF Signal
            self.call(False)
        else:
            # Send ON Signal
            self.call(True)

    def call(self, cmd):
        # Service call
        resp = self.service_connection.call(cmd)            
        if resp.success:
            # Callback (In button event)
            if not(self.callback is None):
                self.callback(cmd)

            if cmd:
                self.setStyleSheet("QPushButton { background: rgb(71, 255, 62) }")
                self.setText("Kill " + self.button_name)
                self.enabled = True
            else:
                self.setStyleSheet("")
                self.setText("Start " + self.button_name)
                self.enabled = False
        else:
            self.setStyleSheet("QPushButton { background: red }")
            self.setText(self.button_name + " : Fail")

    def service_error_cb(self, ex):
        self.setStyleSheet("QPushButton { background: red }")
        self.setText(self.button_name + ": Error")
        rospy.logwarn (self.module_name + " : Exception:" + str(ex))

    def service_connected_cb(self):
        self.setText("Start " + self.button_name)
        self.setStyleSheet("")

    class ServiceConnection(QThread):
        def __init__(self, service_name, error_cb, connected_cb):
            """
                Service connection class
            """
            QThread.__init__(self)
            self.service_name = service_name
            
            self.connected = False
            self.error_callback = error_cb
            self.connected_callback = connected_cb

            self.service = rospy.ServiceProxy(self.service_name, SetBool)

            # Reconnection timer
            self.timer = QTimer()
            self.timer.setInterval(int(5000))
            self.timer.timeout.connect(self.timer_callback)
            # self.timer.start()

            # Start thread
            self.start()

        def call(self, cmd):
            result = SetBool._response_class()
            result.success = False
            try:
                result = self.service(cmd)
            except Exception as ex:
                self.connected = False
                self.error_callback(ex)
                rospy.logwarn("Service disconnected. Waiting for reconnection!")
            return result

        def stop_connection(self):
            self.timer.stop()

        def __del__(self):
            self.timer.stop()

        def timer_callback(self):
            # Periodic check for service availability (short check)
            try:
                rospy.wait_for_service(self.service_name, timeout=0.01)
                if not self.connected:
                    # Successfull connection
                    self.connected = True
                    self.connected_callback()
            except Exception as ex:
                self.connected = False
                self.error_callback(ex)


        def run(self):
            try:
                rospy.wait_for_service(self.service_name, timeout=3)
                if not self.connected:
                    # Successfull connection
                    self.connected = True
                    self.connected_callback()
            except Exception as ex:
                self.connected = False
                self.error_callback(ex)
            

