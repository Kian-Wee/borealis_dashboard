import rospy
from std_srvs.srv import SetBool

import PyQt5 as qt
from PyQt5.QtWidgets import QPushButton

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
        
        self.service = None
        try:
            rospy.wait_for_service(self.service_name, timeout=2)
            self.service = rospy.ServiceProxy(self.service_name, SetBool)
        except Exception as e:
            self.setStyleSheet("QPushButton { background: red }")
            self.setText(self.button_name + ": Error")
            rospy.logwarn (self.module_name + " : Exception:" + str(e))

        self.clicked.connect(self.button_press)

    def button_press(self):
        if self.enabled:
            # Kill Service
            cmd = False
        else:
            # Start Service
            cmd = True
        # Execute command
        self.call(cmd)

    def call(self, cmd):
        # Service call
        if not (self.service == None):
            resp = self.service(cmd)
            if resp.success:
                # Callback (In button event)
                self.callback(cmd)

                if self.cmd:
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
        else:
            self.setStyleSheet("QPushButton { background: red }")
            self.setText(self.button_name + ": Error")
            rospy.logwarn(self.module_name + " : " + self.button_name + " Service Not Available")
        