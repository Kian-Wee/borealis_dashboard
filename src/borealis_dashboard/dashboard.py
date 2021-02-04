import os
import sys
import rospy
import rospkg
import rosparam
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
import PyQt5 as qt

from uav_diagnostics import UAV_Diagnostic
from human_diagnostics import Human_Diagnostic
from experiment_control import ControlCenter

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
        ui_file = os.path.join(rospkg.RosPack().get_path('borealis_dashboard'), 'resource', 'Form.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DashboardUi')

        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Load parameters
        self.load_parameters("parameters")

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        indicators_layout = qt.QtWidgets.QHBoxLayout()
        self.control_center = ControlCenter(indicators_layout, self.start_experiment)
        self.human = Human_Diagnostic(indicators_layout, footIMU_topic="footIMU/IMU", odometry_topic="imu_odometry", odometry_service="/human_ros_launcher/odometry", fusion_service="/human_ros_launcher/fusion")
        self.uav1 = UAV_Diagnostic(indicators_layout, left_topic='/UAV1/UAV1_left', right_topic='/UAV1/UAV1_right', odom_topic="/camera_1/odom/sample", uav_name='UAV1', uwb_service="/uav1_ros_launcher/uwb")
        self.uav2 = UAV_Diagnostic(indicators_layout, left_topic='/UAV2/UAV2_left', right_topic='/UAV2/UAV2_right', odom_topic="/camera_2/odom/sample", uav_name='UAV2', uwb_service="/uav2_ros_launcher/uwb")
        # self.uav3 = UAV_Diagnostic(indicators_layout, left_topic='/UAV3/UAV3_left', right_topic='/UAV3/UAV3_right', odom_topic="/camera_3/odom/sample", uav_name='UAV3')
        
        button_layout = qt.QtWidgets.QVBoxLayout()
        exit_button = qt.QtWidgets.QPushButton("Exit")
        exit_button.setStyleSheet("QPushButton { background: qlineargradient(spread:pad, x1:1, y1:1, x2:1, y2:0, stop:0 rgba(255, 0, 0, 215), stop:1 rgba(255, 255, 255, 255)) }")

        button_layout.addWidget(exit_button)

        # Connect Signals
        exit_button.clicked.connect(self.exit)

        dashboard_layout = qt.QtWidgets.QVBoxLayout()
        dashboard_layout.addLayout(indicators_layout)
        dashboard_layout.addLayout(button_layout)

        self._widget.setLayout(dashboard_layout)
    
    def load_parameters(self, file_name):
        try:
            param_file = open(rospkg.RosPack().get_path('borealis_dashboard')+"/config/" + file_name + ".yaml")
            yaml_file = yaml.load(param_file)
            param_file.close()
            rosparam.upload_params(rospy.get_name()+'/', yaml_file)
        except Exception as ex:
            rospy.logerr(str(ex))

    def start_experiment(self, cmd):
        self.human.start(cmd)
        # No need to restart UWB each time experiment starts. Hence ignore experiment stop commands.
        if cmd:
            self.uav1.start(cmd)
            self.uav2.start(cmd)

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

    def exit(self):
        qm = qt.QtWidgets.QMessageBox
        ret = qm.question(self._widget,'Exit Dashboard', "Are you sure you want to Exit?", qm.Yes | qm.No)
        if ret == qm.Yes:
            print "Shutdown Request"
            rospy.signal_shutdown("User Request")
            sys.exit()
    
