import os
import sys
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
import PyQt5 as qt

from uav_diagnostics import UAV_Diagnostic
from human_diagnostics import Human_Diagnostic

class Dashboard(Plugin):

    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Dashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # print (type(context))
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = qt.QtWidgets.QWidget()
        # self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_diagnostic_exp'), 'resource', 'Form.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DashboardUi')

        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Attributes
        self.record = False

        # Fetch GUI Components
        # self.record_all_button = self._widget.findChild(qt.QtWidgets.QPushButton, 'record_all_pushButton')
        # self.clear_button = self._widget.findChild(qt.QtWidgets.QPushButton, 'clear_pushButton')
        # self.exit_button = self._widget.findChild(qt.QtWidgets.QPushButton, 'exit_pushButton')

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        indicators_layout = qt.QtWidgets.QHBoxLayout()
    
        self.human = Human_Diagnostic(indicators_layout, footIMU_topic="footIMU/IMU", waistIMU_topic="waistIMU/IMU", odometry_topic="imu_odometry")
        # self.uav1 = UAV_Diagnostic(layout, left_topic='/UAV1/UAV1_left', right_topic='/UAV1/UAV1_right', odom_topic="/uav1/odom", uav_name='UAV1')
        # self.uav2 = UAV_Diagnostic(layout, left_topic='/UAV2/UAV2_left', right_topic='/UAV2/UAV2_right', odom_topic="/uav2/odom", uav_name='UAV2')
        # self.uav3 = UAV_Diagnostic(layout, left_topic='/UAV3/UAV3_left', right_topic='/UAV3/UAV3_right', odom_topic="/uav3/odom", uav_name='UAV3')
        self.uav1 = UAV_Diagnostic(indicators_layout, left_topic='/UAV1/UAV1_left', right_topic='/UAV1/UAV1_right', odom_topic="/camera_1/odom/sample", uav_name='UAV1')
        self.uav2 = UAV_Diagnostic(indicators_layout, left_topic='/UAV2/UAV2_left', right_topic='/UAV2/UAV2_right', odom_topic="/camera_2/odom/sample", uav_name='UAV2')
        self.uav3 = UAV_Diagnostic(indicators_layout, left_topic='/UAV3/UAV3_left', right_topic='/UAV3/UAV3_right', odom_topic="/camera_3/odom/sample", uav_name='UAV3')
        
        button_layout = qt.QtWidgets.QVBoxLayout()
        exit_button = qt.QtWidgets.QPushButton("Exit")
        clear_button = qt.QtWidgets.QPushButton("Clear")

        button_layout.addWidget(clear_button)
        button_layout.addWidget(exit_button)

        # Connect Signals
        # self.record_all_button.clicked.connect(self.record_all)
        clear_button.clicked.connect(self.clear)
        exit_button.clicked.connect(self.exit)

        dashboard_layout = qt.QtWidgets.QVBoxLayout()
        dashboard_layout.addLayout(indicators_layout)
        dashboard_layout.addLayout(button_layout)

        self._widget.setLayout(dashboard_layout)
    

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def clear(self):
        qm = qt.QtWidgets.QMessageBox
        ret = qm.question(self._widget,'Clear Values', "Are you sure to reset all the values?", qm.Yes | qm.No)
        if ret == qm.Yes:
            self.uav1.clear()
            self.uav2.clear()
            self.uav3.clear()
            self.human.clear()

    def exit(self):
        qm = qt.QtWidgets.QMessageBox
        ret = qm.question(self._widget,'Exit Dashboard', "Are you sure you want to Exit?", qm.Yes | qm.No)
        if ret == qm.Yes:
            print "Shutdown Request"
            rospy.signal_shutdown("User Request")
            sys.exit()
    
    def record_all(self):
        self.record = not self.record
        if self.record != self.uav1.record:
            self.uav1.record_button.click()
        if self.record != self.uav2.record:
            self.uav2.record_button.click()
        if self.record != self.uav3.record:
            self.uav3.record_button.click()
        if self.record != self.human.record:
            self.human.record_button.click()

        if self.record:
            self.record_all_button.setStyleSheet("QPushButton { background: qlineargradient(spread:pad, x1:1, y1:1, x2:1, y2:0, stop:0 rgba(255, 0, 0, 215), stop:1 rgba(255, 255, 255, 255)) }")
            self.record_all_button.setText("Stop All")
        else:
            self.record_all_button.setStyleSheet("")
            self.record_all_button.setText("Record All")
