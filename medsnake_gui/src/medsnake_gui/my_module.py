import os
import rospy
import rosnode
import rospkg
import datetime
from time import time, sleep
import numpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon
from std_msgs.msg import Char, String
from medical_snake.msg import Tension_readings
from sensor_msgs.msg import Joy 

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('medsnake_gui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # print(self._widget.find(3))
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Connect widget buttons to event handle functions
        # Steering button
        self._widget.steer_left.clicked[bool].connect(self.handle_left_clicked)
        self._widget.steer_right.clicked[bool].connect(self.handle_right_clicked)
        self._widget.steer_up.clicked[bool].connect(self.handle_up_clicked)
        self._widget.steer_down.clicked[bool].connect(self.handle_down_clicked)
        
        # Outer snake button
        self._widget.tighten_outer.clicked[bool].connect(self.handle_tighten_outer_clicked)
        self._widget.loosen_outer.clicked[bool].connect(self.handle_loosen_outer_clicked)
        
        # Inner snake button
        self._widget.tighten_inner.clicked[bool].connect(self.handle_tighten_inner_clicked)
        self._widget.loosen_inner.clicked[bool].connect(self.handle_loosen_inner_clicked)
        
        # Advance and retract
        self._widget.advance.clicked[bool].connect(self.handle_advance_clicked)
        self._widget.retract.clicked[bool].connect(self.handle_retract_clicked)
        
        # Stop and demorospy.Sub(self.handle_tighten_outer_A_clicked)
        self._widget.tighten_outer_B.clicked[bool].connect(self.handle_tighten_outer_B_clicked)
        self._widget.tighten_outer_C.clicked[bool].connect(self.handle_tighten_outer_C_clicked)
        
        self._widget.loosen_outer_A.clicked[bool].connect(self.handle_loosen_outer_A_clicked)
        self._widget.loosen_outer_B.clicked[bool].connect(self.handle_loosen_outer_B_clicked)
        self._widget.loosen_outer_C.clicked[bool].connect(self.handle_loosen_outer_C_clicked)
        
        # Railing
        self._widget.forward_outer.clicked[bool].connect(self.handle_forward_outer_clicked)
        self._widget.backward_outer.clicked[bool].connect(self.handle_backward_outer_clicked)
        
        self._widget.forward_inner.clicked[bool].connect(self.handle_forward_inner_clicked)
        self._widget.backward_inner.clicked[bool].connect(self.handle_backward_inner_clicked)
        # self._widget.home_snake.clicked[bool].connect(self.homing_snake)
        
        # Set up a publisher for the gui_commands
        self.pub_ = rospy.Publisher('/gui_commands', Char, queue_size=1)
        self.rate = rospy.Rate(10)

        # This line is establishing the subscriber for the medsnake mode, passed as a string to snake_mode_define function
        
        self.medsnake_mode_sub_ = rospy.Subscriber('/medsnake_mode', String, self.snake_mode_cb)
        self.tension_reading_sub_ = rospy.Subscriber('/tension_readings', Tension_readings, self.snake_tension_cb)
        self.joystick_sub_ = rospy.Subscriber('/joy', Joy, self.joystick_data_cb) 
                      
    def snake_mode_cb(self, data):
        self._widget.snake_mode.setText(data.data)
        if data.data not "Snake is Ready":
            self._widget.snake_mode.setStyleSheet("background-color: red")
        self.joystick_active_cb()
            
    def joystick_active_cb(self):
        if "/joy_node" in rosnode.get_node_names():
            self._widget.joy_connected.setText("Active")
        else:
            self._widget.joy_connected.setText("Inactive")

    # self.tension_array = []
    def snake_tension_cb(self, tension):
        # global tension_array
        self._widget.inner_tension.setText(str(round(tension.inner_snake_cable, 2)) + " N")
        self._widget.outer_tension_a.setText(str(round(tension.outer_snake_cable_A, 2)) + " N")
        self._widget.outer_tension_b.setText(str(round(tension.outer_snake_cable_B, 2)) + " N")
        self._widget.outer_tension_c.setText(str(round(tension.outer_snake_cable_C, 2)) + " N")
        
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
    
    # Steering event handle
    def handle_left_clicked(self):
        data = 97 # a
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_right_clicked(self):
        data = 100 # d
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_up_clicked(self):
        data = 121 # y
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    def handle_down_clicked(self):
        data = 104 # h
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    # Outer snake event handle
    def handle_tighten_outer_clicked(self):
        data = 116 # t
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_loosen_outer_clicked(self):
        data = 103 # g
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)

    # Outer snake event handle
    def handle_tighten_inner_clicked(self):
        data = 118 # v
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_loosen_inner_clicked(self):
        data = 98 # b
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Advance and retract event handle
    def handle_advance_clicked(self):
        data = 119 # w
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_retract_clicked(self):
        data = 115 # s
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Stop and demo event handle
    def handle_stop_clicked(self):
        data = 111 # o
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_demo_clicked(self):
        data = 113 # q
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
          
    
    # Individual outer cable event handle
    def handle_tighten_outer_A_clicked(self):
        data = 105 # i
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)

    def handle_tighten_outer_B_clicked(self):
        data = 107 # k
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_tighten_outer_C_clicked(self):
        data = 106 # j
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)  ros::init(argc, argv, "medsnake_main_node");


    def handle_loosen_outer_B_clicked(self):
        data = 108 # l  File "/home/biomed/julius/medsnake_julius_ws/src/medsnake_gui/src/medsnake_gui/my_module.py", line 279

        
    def handle_loosen_outer_C_clicked(self):
        data = 110 # n
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Railing event handle
    def handle_forward_outer_clicked(self):
        data = 117 # u
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_backward_outer_clicked(self):
        data = 109 # m
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)

    def handle_forward_inner_clicked(self):
        data = 101 # e
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_backward_inner_clicked(self):
        data = 99 # c
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    def joystick_data_cb(self, joystick_data):
        # Updating GUI widget
        self.x_pos = joystick_data.axes[0]
        self.y_pos = joystick_data.axes[1]
        
        # Calculating the angle of the joystick position, accounting for arctan range being -90 to 90
        
        self.joy_angle = self.determine_joy_angle([self.x_pos, -self.y_pos])
        self._widget.joy_angle.setText(str(self.joy_angle)+"°")
        self._widget.joystickKnob.move(round((-self.x_pos+1.2)*30.5, 2), round((-self.y_pos+1.2)*30.5, 2))
        
        # Running functions at joystick unit vector positions, cases (0, 1), (1, 0), (-1, 0), (0, -1)
        
        if self.x_pos == 0 and self.y_pos == 1:
            # moving down
            self.handle_up_clicked()
        elif self.x_pos == 1 and self.y_pos == 0:
            # moving left
            self.handle_left_clicked()
        elif self.x_pos == 0 and self.y_pos == -1:
            # moving up
            self.handle_down_clicked()
        elif self.x_pos == -1 and self.y_pos == 0:
            # moving right
            self.handle_right_clicked()
            
        # Checking for buttons pressed down
        
        # Stop
        if joystick_data.buttons[9] == 1:
            self.handle_stop_clicked()
            
        # Advance
        if joystick_data.buttons[6] == 1:
            self.handle_advance_clicked()
        
        # Retract 
        if joystick_data.buttons[4] == 1:
            self.handle_retract_clicked()
        
    
    def determine_joy_angle(self, position_array):
        # Function determines joystick angle by taking x and y positions of joystick, then arctan
        # x position must be first input array element, y position second
        # Returned value is rounded to 1st decimal place
        if position_array[0] == 0 and position_array[1] == 0:
            return 0
        else:
            return round((numpy.arctan2(position_array[1], position_array[0]) * (180/numpy.pi)) + 180, 1)

        
    
        