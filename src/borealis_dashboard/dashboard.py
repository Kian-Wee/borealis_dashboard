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

# TODO: fix missing flight mode bug on uav3

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
        print("Borealis Dashboard Loaded")
        
        indicators_layout = qt.QtWidgets.QHBoxLayout()
        self.control_center = ControlCenter(indicators_layout, self.start_experiment)

        # Get Parameters
        # Where rospy.get_param('default_param to pull from', 'default_topic')
        self.human_foot_imu_topic = rospy.get_param(rospy.get_name()+ '/human_foot_imu_topic', '/footIMU/IMU')
        self.human_odometry_topic = rospy.get_param(rospy.get_name()+ '/human_odometry_topic', '/imu_odometry')
        # self.human_uwb_topic = rospy.get_param(rospy.get_name()+ '/human_uwb_topic', '/HumanPose')
        self.human_uwb_topic = '/HumanPose'
        self.human_odometry_enable_service = rospy.get_param(rospy.get_name()+ '/human_odometry_enable_service', '/human_ros_launcher/odometry')
        self.human_odometry_fusion_enable_service = rospy.get_param(rospy.get_name()+ '/human_odometry_fusion_enable_service', '/human_ros_launcher/fusion')
        self.human_glove_enable_service = rospy.get_param(rospy.get_name()+ '/human_glove_enable_service', '/human_ros_launcher/glove')
        self.human_gun_enable_service = rospy.get_param(rospy.get_name()+ '/human_gun_enable_service', '/human_ros_launcher/rifle')
        self.human_hri_enable_service = rospy.get_param(rospy.get_name()+ '/human_hri_enable_service', '/human_ros_launcher/hri')
        self.human_drone_yaw_control_enable_service = rospy.get_param(rospy.get_name()+ '/human_drone_yaw_control_enable_service', '/human_ros_launcher/drone_yaw_control')
        self.human_uwb_enable_service = rospy.get_param(rospy.get_name()+ '/human_uwb_enable_service', '/human_ros_launcher/uwb')

        self.uav1_uwb_topic = rospy.get_param(rospy.get_name()+ '/uav1_uwb_topic', '/UAV1/UWB')
        self.uav1_odometry_topic = rospy.get_param(rospy.get_name()+ '/uav1_odometry_topic', '/camera_1/odom/sample')
        self.uav1_target_topic = rospy.get_param(rospy.get_name()+ '/uav1_target_topic', '/uav1/mavros/setpoint_position/local') # '/uav1/command/pose'
        self.uav1_target_yaw_topic = rospy.get_param(rospy.get_name()+ '/uav1_target_yaw_topic', '/uav1/command/yaw')
        self.uav1_uwb_enable_service = rospy.get_param(rospy.get_name()+ '/uav1_uwb_enable_service', '/uav1_ros_launcher/uwb')
        self.uav1_datafeed_enable_service = rospy.get_param(rospy.get_name()+ '/uav1_datafeed_enable_service', '/uav1_ros_launcher/datafeed')
        self.uav1_local_position_topic = rospy.get_param(rospy.get_name()+ '/uav1/mavros/local_position/pose', '/uav1/mavros/local_position/odom') # Local position as given by mavros
        self.uav1_uav_state_status_topic = rospy.get_param(rospy.get_name()+ '/uav1/mavros/state', '/uav1/mavros/state') # For Flight Modes
        self.uav1_uav_battery_status_topic = rospy.get_param(rospy.get_name()+ '/uav1/mavros/battery', '/uav1/mavros/battery') # For Battery
        self.uav1_uav_mode_topic = rospy.get_param(rospy.get_name()+ '/uav1/hri_mode', '/uav1/hri_mode') # For Follow, Go
        self.uav1_planner_status_topic = rospy.get_param(rospy.get_name()+ '/uav1/planner_status', '/uav1/planner_status')

        self.uav1_teaming_planner_rf_phasesync_topic = "/uav1/borealis_teaming_planner/phase_and_time_rf"
        self.uav1_teaming_planner_cp_status_topic = "/uav1/borealis_teaming_planner/phase_and_time_cp"

        self.uav2_uwb_topic = rospy.get_param(rospy.get_name()+ '/uav2_uwb_topic', '/UAV2/UWB')
        self.uav2_odometry_topic = rospy.get_param(rospy.get_name()+ '/uav2_odometry_topic', '/camera_2/odom/sample')
        self.uav2_target_topic = rospy.get_param(rospy.get_name()+ '/uav2_target_topic', '/uav2/mavros/setpoint_position/local')
        self.uav2_target_yaw_topic = rospy.get_param(rospy.get_name()+ '/uav2_target_yaw_topic', '/uav2/command/yaw')
        self.uav2_uwb_enable_service = rospy.get_param(rospy.get_name()+ '/uav2_uwb_enable_service', '/uav2_ros_launcher/uwb')
        self.uav2_datafeed_enable_service = rospy.get_param(rospy.get_name()+ '/uav2_datafeed_enable_service', '/uav2_ros_launcher/datafeed')
        self.uav2_local_position_topic = rospy.get_param(rospy.get_name()+ '/uav2/mavros/local_position/pose', '/uav2/mavros/local_position/odom') # Local position as given by mavros
        self.uav2_uav_state_status_topic = rospy.get_param(rospy.get_name()+ '/uav2/mavros/state', '/uav2/mavros/state') # For Flight Modes
        self.uav2_uav_battery_status_topic = rospy.get_param(rospy.get_name()+ '/uav2/mavros/battery', '/uav2/mavros/battery') # For Battery
        self.uav2_uav_mode_topic = rospy.get_param(rospy.get_name()+ '/uav2/hri_mode', '/uav2/hri_mode') # For Follow, Go
        self.uav2_planner_status_topic = rospy.get_param(rospy.get_name()+ '/uav2/planner_status', '/uav2/planner_status')

        self.uav2_teaming_planner_rf_phasesync_topic = "/uav2/borealis_teaming_planner/phase_and_time_rf"
        self.uav2_teaming_planner_cp_status_topic = "/uav2/borealis_teaming_planner/phase_and_time_cp"


        self.uav3_uwb_topic = rospy.get_param(rospy.get_name()+ '/uav3_uwb_topic', '/UAV3/UWB')
        self.uav3_odometry_topic = rospy.get_param(rospy.get_name()+ '/uav3_odometry_topic', '/camera_3/odom/sample')
        self.uav3_target_topic = rospy.get_param(rospy.get_name()+ '/uav3_target_topic', '/uav3/mavros/setpoint_position/local')
        self.uav3_target_yaw_topic = rospy.get_param(rospy.get_name()+ '/uav3_target_yaw_topic', '/uav3/command/yaw')
        self.uav3_uwb_enable_service = rospy.get_param(rospy.get_name()+ '/uav3_uwb_enable_service', '/uav3_ros_launcher/uwb')
        self.uav3_datafeed_enable_service = rospy.get_param(rospy.get_name()+ '/uav3_datafeed_enable_service', '/uav3_ros_launcher/datafeed')
        self.uav3_local_position_topic = rospy.get_param(rospy.get_name()+ '/uav3/mavros/local_position/pose', '/uav3/mavros/local_position/odom') # Local position as given by mavros
        self.uav3_uav_state_status_topic = rospy.get_param(rospy.get_name()+ '/uav3/mavros/state', '/uav3/mavros/state') # For Flight Modes
        self.uav3_uav_battery_status_topic = rospy.get_param(rospy.get_name()+ '/uav3/mavros/battery', '/uav3/mavros/battery') # For Battery
        self.uav3_uav_mode_topic = rospy.get_param(rospy.get_name()+ '/uav3/hri_mode', '/uav3/hri_mode') # For Follow, Go
        self.uav3_planner_status_topic = rospy.get_param(rospy.get_name()+ '/uav3/planner_status', '/uav3/planner_status')

        self.uav3_teaming_planner_rf_phasesync_topic = "/uav3/borealis_teaming_planner/phase_and_time_rf"
        self.uav3_teaming_planner_cp_status_topic = "/uav3/borealis_teaming_planner/phase_and_time_cp"
        self.uav1_ping_server = "192.168.1.62"
        self.uav2_ping_server = "192.168.1.63"
        self.uav3_ping_server = "192.168.1.64"


        # Create GUI Modules
        self.human = Human_Diagnostic(indicators_layout, 
                                        footIMU_topic=self.human_foot_imu_topic, 
                                        odometry_topic=self.human_odometry_topic,
                                        uwb_topic=self.human_uwb_topic, 
                                        odometry_service=self.human_odometry_enable_service, 
                                        fusion_service=self.human_odometry_fusion_enable_service,
                                        glove_service=self.human_glove_enable_service,
                                        gun_service=self.human_gun_enable_service,
                                        hri_service=self.human_hri_enable_service,
                                        drone_yaw_control_service=self.human_drone_yaw_control_enable_service,
                                        uwb_service=self.human_uwb_enable_service)

        self.uav1 = UAV_Diagnostic(indicators_layout, 
                                    uwb_topic=self.uav1_uwb_topic,
                                    odom_topic=self.uav1_odometry_topic, 
                                    uav_name='UAV1', 
                                    uwb_service=self.uav1_uwb_enable_service,
                                    datafeed_service=self.uav1_datafeed_enable_service, 
                                    target_topic=self.uav1_target_topic,
                                    local_position_topic=self.uav1_local_position_topic, uwb_position_topic="/UAV1PoseUWB",
                                    uav_state_status_topic=self.uav1_uav_state_status_topic,
                                    uav_battery_status_topic=self.uav1_uav_battery_status_topic,
                                    uav_mode_topic=self.uav1_uav_mode_topic,
                                    planner_status_topic= self.uav1_planner_status_topic,

                                    teaming_planner_rf_phasesync_topic = self.uav1_teaming_planner_rf_phasesync_topic,
                                    teaming_planner_cp_status_topic = self.uav1_teaming_planner_cp_status_topic,
                                    ping_server = self.uav1_ping_server
                                    )

        self.uav2 = UAV_Diagnostic(indicators_layout, 
                                    uwb_topic=self.uav2_uwb_topic,
                                    odom_topic=self.uav2_odometry_topic, 
                                    uav_name='UAV2', 
                                    uwb_service=self.uav2_uwb_enable_service,
                                    datafeed_service=self.uav2_datafeed_enable_service, 
                                    target_topic=self.uav2_target_topic,
                                    local_position_topic=self.uav2_local_position_topic, uwb_position_topic="/UAV2PoseUWB",
                                    uav_state_status_topic=self.uav2_uav_state_status_topic,
                                    uav_battery_status_topic=self.uav2_uav_battery_status_topic,
                                    uav_mode_topic=self.uav2_uav_mode_topic,
                                    planner_status_topic= self.uav2_planner_status_topic,

                                    teaming_planner_rf_phasesync_topic = self.uav2_teaming_planner_rf_phasesync_topic,
                                    teaming_planner_cp_status_topic = self.uav2_teaming_planner_cp_status_topic,
                                    ping_server = self.uav2_ping_server
                                    )


        self.uav3 = UAV_Diagnostic(indicators_layout, 
                                    uwb_topic=self.uav3_uwb_topic,
                                    odom_topic=self.uav3_odometry_topic, 
                                    uav_name='UAV3', 
                                    uwb_service=self.uav3_uwb_enable_service,
                                    datafeed_service=self.uav3_datafeed_enable_service, 
                                    target_topic=self.uav3_target_topic,
                                    local_position_topic=self.uav3_local_position_topic, uwb_position_topic='/UAV3PoseUWB',
                                    uav_state_status_topic=self.uav3_uav_state_status_topic,
                                    uav_battery_status_topic=self.uav3_uav_battery_status_topic,
                                    uav_mode_topic=self.uav3_uav_mode_topic,
                                    planner_status_topic= self.uav3_planner_status_topic,

                                    teaming_planner_rf_phasesync_topic = self.uav3_teaming_planner_rf_phasesync_topic,
                                    teaming_planner_cp_status_topic = self.uav3_teaming_planner_cp_status_topic,
                                    ping_server = self.uav3_ping_server
                                    )

        
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

        rospy.Rate(2) # Run at lower rate to prevent bandwidth saturation
    
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
            self.uav3.start(cmd)

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
    
