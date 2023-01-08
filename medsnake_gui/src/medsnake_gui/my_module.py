import os
import rospy
import rosnode
import rospkg
import datetime
import numpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon
from std_msgs.msg import Char, String
from medical_snake.msg import Tension_readings
from sensor_msgs.msg import Joy 

import sys
from time import sleep

from PyQt5.QtCore import QRunnable, Qt, QThreadPool, QThread, QObject, pyqtSignal

from PyQt5.QtWidgets import (

    QApplication,

    QLabel,

    QMainWindow,

    QPushButton,

    QVBoxLayout,

    QWidget,

)


# Step 1: Create a worker class
class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def run(self):
        """Long-running task."""
        while not rospy.is_shutdown():
            sleep(1/15)
            self.progress.emit(1)
        self.finished.emit()

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
        self._widget.tighten_outer_A.clicked[bool].connect(self.handle_tighten_outer_A_clicked)
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
        self._widget.home.clicked[bool].connect(self.handle_homing_clicked)
        
        # Demo
        self._widget.demo.clicked[bool].connect(self.handle_demo_clicked)

        
        # Set up a publisher for the gui_commands
        self.pub_ = rospy.Publisher('/gui_commands', Char, queue_size=1)
        self.rate = rospy.Rate(10)

        # This line is establishing the subscriber for the medsnake mode, passed as a string to snake_mode_define function
        
        self.medsnake_mode_sub_ = rospy.Subscriber('/medsnake_mode', String, self.snake_mode_cb)
        self.tension_reading_sub_ = rospy.Subscriber('/tension_readings', Tension_readings, self.snake_tension_cb)
        self.joystick_sub_ = rospy.Subscriber('/joy', Joy, self.joystick_data_cb) 
        
        self.snake_mode_temp = "Loading..."
        self.joy_angle = 0
        self.inner_tension_temp = 0
        self.outer_tension_a_temp = 0
        self.outer_tension_b_temp = 0
        self.outer_tension_c_temp = 0
        self.x_pos = 0
        self.y_pos = 0
        self.joystick_button_9_temp = 0
        self.joystick_button_6_temp = 0
        self.joystick_button_4_temp = 0
        self.joystick_button_7_temp = 0
        
        
        # Threading variables
        
        # # Step 2: Create a QThread object
        # self.thread = QThread()
        # # Step 5: Connect signals and slots
        # self.thread.started.connect(self.update_loop)
        # self.thread.start()
        
        # Step 2: Create a QThread object
        self.thread = QThread()
        # Step 3: Create a worker object
        self.worker = Worker()
        # Step 4: Move worker to the thread
        self.worker.moveToThread(self.thread)
        # Step 5: Connect signals and slots
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.progress.connect(self.update_gui)
        # Step 6: Start the thread
        
        self.thread.start()
            
    def update_gui(self):
        
        self._widget.snake_mode.setText(self.snake_mode_temp)
        if self.snake_mode_temp != "Snake is Ready":
            self._widget.snake_mode.setStyleSheet("background-color: red")
        else:
            self._widget.snake_mode.setStyleSheet("background-color: white")
        
        # Checking if joystick is still connected
        if "/joy_node" in rosnode.get_node_names():
            self._widget.joy_connected.setText("Active")
            self._widget.joy_connected.setStyleSheet("background-color: green")
        else:
            self._widget.joy_connected.setText("Inactive")
            self._widget.joy_connected.setStyleSheet("background-color: red")
       
        # Setting the current joystick angle
        self.joy_angle = self.determine_joy_angle([self.x_pos, -self.y_pos])
        self._widget.joy_angle.setText(str(self.joy_angle)+"°")
        self._widget.joystickKnob.move(round((-self.x_pos+1.2)*30.5, 2), round((-self.y_pos+1.2)*30.5, 2))
            
        # Stop
        if self.joystick_button_9_temp == 1:
            self.handle_stop_clicked()
                
        # Advance
        if self.joystick_button_6_temp == 1:
            self.handle_advance_clicked()
            
        # Retract 
        if self.joystick_button_4_temp == 1:
            self.handle_retract_clicked()
            
        # # Demo
        # if self.joystick_button_7_temp == 1:
        #     self.handle_demo_clicked()
                
        self._widget.inner_tension.setText(str(round(self.inner_tension_temp, 2)) + " N")
        self._widget.outer_tension_a.setText(str(round(self.outer_tension_a_temp, 2)) + " N")
        self._widget.outer_tension_b.setText(str(round(self.outer_tension_b_temp, 2)) + " N")
        self._widget.outer_tension_c.setText(str(round(self.outer_tension_c_temp, 2)) + " N")            
              
    def snake_mode_cb(self, data):
        self.snake_mode_temp = data.data 

    # self.tension_array = []
    def snake_tension_cb(self, tension):
        # global tension_array
        self.inner_tension_temp = tension.inner_snake_cable
        self.outer_tension_a_temp = tension.outer_snake_cable_A
        self.outer_tension_b_temp = tension.outer_snake_cable_B
        self.outer_tension_c_temp = tension.outer_snake_cable_C
                
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
        
          
    def handle_homing_clicked(self):
        data = 114 # r
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
        self.pub_.publish(data)

    def handle_loosen_outer_A_clicked(self):
        data = 112 # p
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    def handle_loosen_outer_B_clicked(self):
        data = 108 # l  
        self.pub_.publish(data)

        
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

        self.joystick_button_4_temp = joystick_data.buttons[4]
        self.joystick_button_6_temp = joystick_data.buttons[6]
        self.joystick_button_7_temp = joystick_data.buttons[7]
        self.joystick_button_9_temp = joystick_data.buttons[9]
    
    def determine_joy_angle(self, position_array):
        # Function determines joystick angle by taking x and y positions of joystick, then arctan
        # x position must be first input array element, y position second
        # Returned value is rounded to 1st decimal place
        if position_array[0] == 0 and position_array[1] == 0:
            return 0
        else:
            return round((numpy.arctan2(position_array[1], position_array[0]) * (180/numpy.pi)) + 180, 1)
    

        
    
        