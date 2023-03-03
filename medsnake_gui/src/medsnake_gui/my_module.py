# File comments last updated by Julius on 02/11/2023

# Import necessary packages
import os
import rospy
import rosnode
import rospkg
from datetime import datetime, timedelta
import numpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon
from std_msgs.msg import Char, String
from medical_snake.msg import Tension_readings, Motor_positions
from sensor_msgs.msg import Joy 
from tkinter import messagebox

import sys
from time import sleep

from PyQt5.QtCore import QRunnable, Qt, QThreadPool, QThread, QObject, pyqtSignal

from PyQt5.QtGui import QFont

from PyQt5 import QtCore

from PyQt5.QtWidgets import (

    QApplication,

    QLabel,

    QMainWindow,

    QPushButton,

    QVBoxLayout,

    QWidget,

)

# Establish 2 worker threads. 
# 1st worker named 'Worker' updates the GUI. 
# 2nd worker named 'Worker1' checks tensions and rail positions in continous
# mode to ensure limits aren't being exceeded. 

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def run(self):
        """Long-running task."""
        while not rospy.is_shutdown():
            sleep(1/15)
            self.progress.emit(1)
        self.finished.emit()
        
class Worker1(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def run(self):
        """Long-running task."""
        while not rospy.is_shutdown():
            sleep(1/15)
            self.progress.emit(1)
        self.finished.emit()

# Define the widget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        
        # Configure the UI file and window
        
        self.setObjectName('MyPlugin')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('medsnake_gui'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MyPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
        # Connect necessary UI events to functions. 
        # Functions with 'clicked' reference actions in 'discrete' mode
        # Functions with 'pressed' and 'released' reference actions in 'continous' mode
        
        # Discrete: steer up, right, left, down
        self._widget.steer_left.clicked[bool].connect(self.handle_left_clicked)
        self._widget.steer_right.clicked[bool].connect(self.handle_right_clicked)
        self._widget.steer_up.clicked[bool].connect(self.handle_up_clicked)
        self._widget.steer_down.clicked[bool].connect(self.handle_down_clicked)
        
        # Discrete: tighen, loosen outer
        self._widget.tighten_outer.clicked[bool].connect(self.handle_tighten_outer_clicked)
        self._widget.loosen_outer.clicked[bool].connect(self.handle_loosen_outer_clicked)
        
        # Discrete: tighen, loosen inner
        self._widget.tighten_inner.clicked[bool].connect(self.handle_tighten_inner_clicked)
        self._widget.loosen_inner.clicked[bool].connect(self.handle_loosen_inner_clicked)
        
        # Discrete & Continous: advance, retract snake
        self._widget.advance.clicked[bool].connect(self.handle_advance_clicked)
        self._widget.retract.clicked[bool].connect(self.handle_retract_clicked)
        
        # Discrete: tighten, loosen outer snake cables A, B, C
        self._widget.tighten_outer_A.clicked[bool].connect(self.handle_tighten_outer_A_clicked)
        self._widget.tighten_outer_B.clicked[bool].connect(self.handle_tighten_outer_B_clicked)
        self._widget.tighten_outer_C.clicked[bool].connect(self.handle_tighten_outer_C_clicked)
        self._widget.loosen_outer_A.clicked[bool].connect(self.handle_loosen_outer_A_clicked)
        self._widget.loosen_outer_B.clicked[bool].connect(self.handle_loosen_outer_B_clicked)
        self._widget.loosen_outer_C.clicked[bool].connect(self.handle_loosen_outer_C_clicked)
        
        # Discrete: forward, backward outer
        self._widget.forward_outer.clicked[bool].connect(self.handle_fwd_outer_clicked)
        self._widget.backward_outer.clicked[bool].connect(self.handle_back_outer_clicked)
        
        # Discrete: forward, backward inner
        self._widget.forward_inner.clicked[bool].connect(self.handle_fwd_inner_clicked)
        self._widget.backward_inner.clicked[bool].connect(self.handle_back_inner_clicked)
        
        # Discrete: forward, backward both
        self._widget.backward_both.clicked[bool].connect(self.handle_back_both_clicked)
        self._widget.forward_both.clicked[bool].connect(self.handle_fwd_both_clicked)
        
        # Discrete: demo, home, stop
        self._widget.home.clicked[bool].connect(self.handle_homing_clicked) 
        self._widget.demo.clicked[bool].connect(self.handle_demo_clicked)
        self._widget.stop.clicked[bool].connect(self.handle_stop_clicked)
        
        # Discrete & Continous: switch between the two modes and compliant insertion
        self._widget.mode_switch.clicked[bool].connect(self.control_mode_clicked)
        self._widget.compliant_insertion.clicked[bool].connect(self.compliant_insertion_clicked)
        
        # Continous: forward both snakes
        self._widget.forward_both.pressed.connect(self.handle_fwd_both_pressed)
        self._widget.forward_both.released.connect(self.continous_mode_button_released)
        
        # Continous: backward both snakes
        self._widget.backward_both.pressed.connect(self.handle_back_both_pressed)
        self._widget.backward_both.released.connect(self.continous_mode_button_released)
        
        # Continous: forward outer
        self._widget.forward_outer.pressed.connect(self.handle_fwd_outer_pressed)
        self._widget.forward_outer.released.connect(self.continous_mode_button_released)
        
        # Continous: backward outer
        self._widget.backward_outer.pressed.connect(self.handle_back_outer_pressed)
        self._widget.backward_outer.released.connect(self.continous_mode_button_released)
        
        # Continous: forward inner
        self._widget.forward_inner.pressed.connect(self.handle_fwd_inner_pressed)
        self._widget.forward_inner.released.connect(self.continous_mode_button_released)
        
        # Continous: backward inner
        self._widget.backward_inner.pressed.connect(self.handle_back_inner_pressed)
        self._widget.backward_inner.released.connect(self.continous_mode_button_released)
        
        # Continous: tighten outer snake cables A, B, C
        self._widget.tighten_outer_A.pressed.connect(self.handle_tighten_outer_A_pressed)
        self._widget.tighten_outer_A.released.connect(self.continous_mode_button_released)
        self._widget.tighten_outer_B.pressed.connect(self.handle_tighten_outer_B_pressed)
        self._widget.tighten_outer_B.released.connect(self.continous_mode_button_released)
        self._widget.tighten_outer_C.pressed.connect(self.handle_tighten_outer_C_pressed)
        self._widget.tighten_outer_C.released.connect(self.continous_mode_button_released)

        # Continous: loosen outer snake cables A, B, C
        self._widget.loosen_outer_A.pressed.connect(self.handle_loosen_outer_A_pressed)
        self._widget.loosen_outer_A.released.connect(self.continous_mode_button_released)
        self._widget.loosen_outer_B.pressed.connect(self.handle_loosen_outer_B_pressed)
        self._widget.loosen_outer_B.released.connect(self.continous_mode_button_released)
        self._widget.loosen_outer_C.pressed.connect(self.handle_loosen_outer_C_pressed)
        self._widget.loosen_outer_C.released.connect(self.continous_mode_button_released)
        
        # Continous: tighten, loosen outer
        self._widget.tighten_outer.pressed.connect(self.handle_tighten_outer_pressed)
        self._widget.tighten_outer.released.connect(self.continous_mode_button_released)
        self._widget.loosen_outer.pressed.connect(self.handle_loosen_outer_pressed)
        self._widget.loosen_outer.released.connect(self.continous_mode_button_released)

        # Continous: tighten, loosen inner
        self._widget.tighten_inner.pressed.connect(self.handle_tighten_inner_pressed)
        self._widget.tighten_inner.released.connect(self.continous_mode_button_released)
        self._widget.loosen_inner.pressed.connect(self.handle_loosen_inner_pressed)
        self._widget.loosen_inner.released.connect(self.continous_mode_button_released)
        
        # Establish publisher for GUI commands
        self.pub_ = rospy.Publisher('/gui_commands', String, queue_size=1)
        self.rate = rospy.Rate(10)

        # Establish subscribers for medsnake mode, tensions, joystick, and rail positions
        self.medsnake_mode_sub_ = rospy.Subscriber('/medsnake_mode', String, self.snake_mode_cb)
        self.tension_reading_sub_ = rospy.Subscriber('/tension_readings', Tension_readings, self.snake_tension_cb)
        self.joystick_sub_ = rospy.Subscriber('/joy', Joy, self.joystick_data_cb) 
        self.rail_pos_sub_ = rospy.Subscriber('/motor_positions', Motor_positions, self.snake_rail_cb) 

        # Declare necessary variables which are updated in callback functions
        # and then updated in interface through update_gui in worker thread #1
        
        # Monitor the live snake mode
        self.snake_mode_temp = "Loading..."

        # Monitor live tensions
        self.inner_tension_temp = 0
        self.outer_tension_a_temp = 0
        self.outer_tension_b_temp = 0
        self.outer_tension_c_temp = 0
        
        # Joystick monitoring variables
        self.x_pos = 0
        self.y_pos = 0
        self.joystick_button_9_temp = 0
        self.joystick_button_6_temp = 0
        self.joystick_button_4_temp = 0
        self.joystick_button_7_temp = 0
        self.joy_angle = 0
        
        # Monitor control mode
        self.control_mode = "Discrete"
        self.joystick_7_pressed = False
        
        # Position-monitoring variables. Range variables are the total motor 
        # position range to go from 0% -> 100% for both snakes. 
        self.inner_snake_pos = 0
        self.outer_snake_pos = 0
        self.inner_snake_range = 12147431
        self.outer_snake_range = 12147320
        self.inner_snake_zero = 0
        self.outer_snake_zero = 0
        
        # Need to watch for the first time snake homes, then turn on rail positions
        self.homed = False
        self.first_homed_adjust = False
        
        # Check if rail positions need to be monitored for rail limit
        self.rail_watch = False
        self.watch_rail = None
        
        # These variables are set up to track the snake as it moves through a gait. 
        self.current_gait = "Initialized"
        self.advance_gait_check = [0, 0, 0, 0, 0, 0]
        self.advance_gait_compare = ["Tightening outer snake to tension ...", "Loosening inner snake...", "Moving inner snake...", 
                                     "Tightening inner snake to tension...", "Loosening outer snake...", "Moving outer snake..."]
        self.retract_gait_check = [0, 0, 0, 0, 0, 0]
        self.retract_gait_compare = ["Moving outer snake...", "Tightening outer snake to tension ...", 
                                     "Loosening inner snake...", "Moving inner snake...", "Tightening inner snake to tension...", "Loosening outer snake..."]
        self.homing_gait_check = [0, 0, 0]
        self.homing_gait_compare = ["Loosening inner snake...", "Loosening outer snake...", "Homing Rail..."]
        self.current_gait_compare = []
        self.current_gait_check = []
        
        # If true, worker thread #2 will monitor tensions to ensure within limits
        self.tension_watch = False
        self.watch_cable = None
        
        # Compliant insertion state monitoring
        self.compliant_insertion_on = False
        
        # Start worker thread #1
        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.progress.connect(self.update_gui)
        self.thread.start()
        
        # Start worker thread #2
        self.thread1 = QThread()
        self.worker1 = Worker()
        self.worker1.moveToThread(self.thread1)
        self.thread1.started.connect(self.worker1.run)
        self.worker1.finished.connect(self.thread1.quit)
        self.worker1.finished.connect(self.worker1.deleteLater)
        self.thread1.finished.connect(self.thread1.deleteLater)
        self.worker1.progress.connect(self.continous_checks)
        self.thread1.start()
        
    def update_gui(self):
        
        # Continously set snake mode text and color
        self._widget.snake_mode.setText(self.snake_mode_temp)
        if self.snake_mode_temp != "Snake is Ready":
            self._widget.snake_mode.setStyleSheet("background-color: red;\nborder: 2px solid black;\nborder-radius: 5px;")
        else:
            self._widget.snake_mode.setStyleSheet("background-color: white;\nborder: 2px solid black;\nborder-radius: 5px;")
        
        # Continously monitor if joystick is connected and active and change color & text
        if "/joy_node" in rosnode.get_node_names():
            self._widget.joy_connected.setText("Active")
            self._widget.joy_connected.setStyleSheet("background-color: green;\nborder: 2px solid black;\nborder-top-right-radius: 5px;\nborder-top-left-radius: 5px;")
        else:
            self._widget.joy_connected.setText("Inactive")
            self._widget.joy_connected.setStyleSheet("background-color: red;\nborder: 2px solid black;\nborder-top-right-radius: 5px;\nborder-top-left-radius: 5px;")
       
        # Update the live joystick angle and location of the joystick in GUI
        self.joy_angle = self.determine_joy_angle([self.x_pos, -self.y_pos])
        self._widget.joy_angle.setText(str(self.joy_angle)+"°")
        self._widget.joystickKnob.move(round((-self.x_pos+1.2)*30.5, 2), round((-self.y_pos+1.2)*30.5, 2))
            
        # Continously monitor joystick input:
        
        # Joystick stop
        if self.joystick_button_9_temp == 1:
            self.handle_stop_clicked()
        # Joystick advance        
        if self.joystick_button_6_temp == 1:
            self.handle_advance_clicked()
        # Joystick retract
        if self.joystick_button_4_temp == 1:
            self.handle_retract_clicked()
            
        if self.joystick_button_7_temp == 1:
            self._widget.compliant_insertion.move(240, 190)
            self._widget.compliant_insertion.setStyleSheet("background-color: rgb(211, 215, 207); border: 2px solid black; border-bottom-right-radius: 5px; border-bottom-left-radius: 5px;")
            self.compliant_insertion_on = True
            if self.control_mode == "Discrete":
                self.control_mode_clicked()
            self.joystick_7_pressed = True
        elif self.joystick_button_7_temp == 0 and self.joystick_7_pressed:
            self._widget.compliant_insertion.move(240, 160)
            self._widget.compliant_insertion.setStyleSheet("background-color: rgb(211, 215, 207); border: 2px solid black; border-top-right-radius: 5px; border-top-left-radius: 5px;")
            self.compliant_insertion_on = False
            self.continous_mode_button_released()
            self.joystick_7_pressed = False
            
            

        # Set live tension readings
        self._widget.inner_tension.setText(str(round(self.inner_tension_temp, 2)) + " N")
        self._widget.outer_tension_a.setText(str(round(self.outer_tension_a_temp, 2)) + " N")
        self._widget.outer_tension_b.setText(str(round(self.outer_tension_b_temp, 2)) + " N")
        self._widget.outer_tension_c.setText(str(round(self.outer_tension_c_temp, 2)) + " N")
        
        # Set live gait status and color, and monitor as snake progresses through gait
        self._widget.snake_gait.setText(self.current_gait)
        if self.current_gait != "Ready":
            if self.current_gait == "ERROR":
                self._widget.snake_gait.setStyleSheet("background-color: red;\nborder: 2px solid black;\nborder-radius: 5px;")
            else:
                self._widget.snake_gait.setStyleSheet("background-color: yellow;\nborder: 2px solid black;\nborder-radius: 5px;")
                
                # 0 in the gait_check means snake has not yet completed step, while 1 means completed
                # Reference cycles are listed in variable declarations, and it continously compares
                if 0 in self.current_gait_check:
                    for j,i in enumerate(self.current_gait_compare):
                        if self.snake_mode_temp == i:
                            self.current_gait_check[j] = 1
                if 0 not in self.current_gait_check and self.snake_mode_temp == "Snake is Ready":
                    if self.current_gait == "Homing":
                        self.homed = True
                        self.inner_snake_zero = self.inner_snake_pos
                    self.outer_snake_zero = self.outer_snake_pos
                    self.current_gait = "Ready"
                    self._widget.snake_gait.setText(self.current_gait)
        else:
            self._widget.snake_gait.setStyleSheet("background-color: white;\nborder: 2px solid black;\nborder-radius: 5px;")
        
        # Check for error state
        if self.snake_mode_temp == "Error":
            self._widget.snake_gait.setText("ERROR")   
            
        # To start the rail monitoring and position readings, snake needs to be homed
        # Once the snake is homed, all of the text and colors will update
        if self.homed:
            self._widget.inner_snake_pos.setText(str(round(self.inner_percent)) + "%")
            self._widget.outer_snake_pos.setText(str(round(self.outer_percent)) + "%")
            self._widget.inner_snake_pos.setAlignment(QtCore.Qt.AlignCenter)
            self._widget.outer_snake_pos.setAlignment(QtCore.Qt.AlignCenter)
            if self.outer_percent < 50:
                self._widget.outer_snake_pos.setStyleSheet("background-color: rgb(78, 154, 6); border-radius: 5px; color: white")
            elif self.outer_percent < 70:
                self._widget.outer_snake_pos.setStyleSheet("background-color: rgb(196, 160, 0); border-radius: 5px; color: white")
            elif self.outer_percent < 90:
                self._widget.outer_snake_pos.setStyleSheet("background-color: rgb(245, 121, 0); border-radius: 5px; color: white")
            else:
                self._widget.outer_snake_pos.setStyleSheet("background-color: red; border-radius: 5px; color: white")
                
            if self.inner_percent < 50:
                self._widget.inner_snake_pos.setStyleSheet("background-color: rgb(78, 154, 6); border-radius: 5px; color: white")
            elif self.outer_percent < 70:
                self._widget.inner_snake_pos.setStyleSheet("background-color: rgb(196, 160, 0); border-radius: 5px; color: white")
            elif self.outer_percent < 90:
                self._widget.inner_snake_pos.setStyleSheet("background-color: rgb(245, 121, 0); border-radius: 5px; color: white")
            else:
                self._widget.inner_snake_pos.setStyleSheet("background-color: red; border-radius: 5px; color: white")
            
        # Once the snake has homed for the first time, QLabels need to be adjusted
        if self.homed and not self.first_homed_adjust:
            self.first_homed_adjust = True
            self._widget.inner_snake_pos.setFont(QFont("Ubuntu Condensed", 11))
            self._widget.outer_snake_pos.setFont(QFont("Ubuntu Condensed", 11))
            self.inner_snake_zero = self.inner_snake_pos
            self.outer_snake_zero = self.outer_snake_pos

    # This is the responsibility of worker #2. continous_checks checks that
    # tension and rail position limits haven't been exceeded
    def continous_checks(self):
        
        # tension_watch is set to True in callback functions when anything to do with tension is initiated.
        if self.tension_watch:
            if self.watch_cable == 'inner':
                if self.inner_tension_temp < 60:
                    pass
                else:
                    data = "stop"
                    self.pub_.publish(data)
                    print("-----------------------tension limit reached-------------------------------------")
                    self.tension_watch = False
                    self.watch_cable = None
            elif self.watch_cable == 'a' or self.watch_cable == 'b' or self.watch_cable == 'c' or self.watch_cable == "outer":
                if self.outer_tension_a_temp < 5 and self.outer_tension_b_temp < 5 and self.outer_tension_c_temp < 5:
                    pass
                else:
                    pass
                    # data = "stop"
                    # self.pub_.publish(data)
                    # self.current_gait = "ERROR"
                    # print("-----------------------tension limit reached-------------------------------------")
                    # self.tension_watch = False
                    # self.watch_cable = None                    
            else:
                self.tension_watch = False
                self.watch_cable = None
                data = "stop"
                self.pub_.publish(data)
                print("-----------------Error in setting continous mode target tension------------------")
                
        # rail_watch is set to True in callback functions when anything to do with railing is initiated. 
        # snake has to be homed for this to be accurate. 
        if self.control_mode == 'Continous' and self.rail_watch and self.homed:
            if self.watch_rail == 'inner':
                if self.inner_percent < -.3 or self.inner_percent > 100:
                    data = "stop"
                    self.pub_.publish(data)
                    print("------------------Inner Rail Limit Reached----------------------")
                    self.rail_watch = False
                    self.watch_rail = None
                elif self.inner_percent > (self.outer_percent) and self.inner_forwards:
                    data = "stop"
                    self.pub_.publish(data)
                    print("------------------Inner Rail Limit Reached----------------------")
                    self.rail_watch = False
                    self.watch_rail = None
                    self.inner_forwards = False
            elif self.watch_rail == 'outer':
                if self.outer_percent < -.3 or self.outer_percent > 100:
                    data = "stop"
                    self.pub_.publish(data)
                    print("------------------Outer Rail Limit Reached----------------------")
                    self.rail_watch = False
                    self.watch_rail = None
                elif self.outer_percent < (self.inner_percent) and self.outer_backwards:
                    data = "stop"
                    self.pub_.publish(data)
                    print("------------------Outer Rail Limit Reached----------------------")
                    self.rail_watch = False
                    self.watch_rail = None
                    self.outer_backwards = False
            elif self.watch_rail == 'both':
                if self.outer_percent < -.3 or self.outer_percent > 100 or self.inner_percent < -.3 or self.inner_percent > 100:
                    data = "stop"
                    self.pub_.publish(data)
                    print("------------------------Rail Limit Reached----------------------")
                    self.rail_watch = False
                    self.watch_rail = None
            else:
                data = "stop"
                self.pub_.publish(data)
                self.rail_watch = False
                self.watch_rail = None
                print("-----------------Error in setting rail position check------------------------------")
                
    # Snake mode subscriber callback func     
    def snake_mode_cb(self, data):
        self.snake_mode_temp = data.data 

    # Snake tension subscriber callback func
    def snake_tension_cb(self, tension):
        # global tension_array
        self.inner_tension_temp = tension.inner_snake_cable
        self.outer_tension_a_temp = tension.outer_snake_cable_A
        self.outer_tension_b_temp = tension.outer_snake_cable_B
        self.outer_tension_c_temp = tension.outer_snake_cable_C
        
    # Snake rail position subscriber callback func
    def snake_rail_cb(self, positions):
        self.inner_snake_pos = positions.inner_snake_motor 
        self.outer_snake_pos = positions.outer_snake_motor
        self.inner_percent = -((self.inner_snake_pos - self.inner_snake_zero) / self.outer_snake_range * 100)
        self.outer_percent = -((self.outer_snake_pos - self.outer_snake_zero) / self.outer_snake_range * 100)
    
    # Joystick subscriber callback function
    def joystick_data_cb(self, joystick_data):
        self.x_pos = joystick_data.axes[0]
        self.y_pos = joystick_data.axes[1]

        self.joystick_button_4_temp = joystick_data.buttons[4]
        self.joystick_button_6_temp = joystick_data.buttons[6]
        self.joystick_button_7_temp = joystick_data.buttons[7]
        self.joystick_button_9_temp = joystick_data.buttons[9]
                
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

    ############################################### BEGIN MOTION CONTROLS #################################################################
    
    # Change control mode if toggle clicked
    def control_mode_clicked(self):
        if self.control_mode == "Discrete":
            self._widget.mode_switch.move(362, 40)
            self._widget.mode_switch.setStyleSheet("border: 2px solid black; background-color: rgb(211, 215, 207); border-bottom-left-radius: 5px; border-top-left-radius: 5px;")
            self.control_mode = "Continous"
            print("---- Control mode has been switched to continous ----")
        elif self.control_mode == "Continous":
            self._widget.mode_switch.move(415, 40)
            self._widget.mode_switch.setStyleSheet("border: 2px solid black; background-color: rgb(211, 215, 207); border-bottom-right-radius: 5px; border-top-right-radius: 5px;")
            self.control_mode = "Discrete"
            print("---- Control mode has been switched to discrete ----")
        else:
            print("---- Error switching control modes ----")
            
    def compliant_insertion_clicked(self):
        if self.compliant_insertion_on:
            self._widget.compliant_insertion.move(240, 160)
            self._widget.compliant_insertion.setStyleSheet("background-color: rgb(211, 215, 207); border: 2px solid black; border-top-right-radius: 5px; border-top-left-radius: 5px;")
            self.compliant_insertion_on = False
            print("---- Compliant insertion on ----")
        elif not self.compliant_insertion_on:
            self._widget.compliant_insertion.move(240, 190)
            self._widget.compliant_insertion.setStyleSheet("background-color: rgb(211, 215, 207); border: 2px solid black; border-bottom-right-radius: 5px; border-bottom-left-radius: 5px;")
            self.compliant_insertion_on = True
            print("---- Compliant insertion off ----")
        else:
            print("---- Error changing compliant insertion mode ----")

    # Discrete: both snakes forward
    def handle_fwd_both_clicked(self):
        if self.control_mode == "Discrete":
            data = 'fwd_both' # x
            self.pub_.publish(data)
    
    # Continous: both snakes forward
    def handle_fwd_both_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "both"
            data = "fwd_both_cont"
            self.pub_.publish(data)

    # Discrete: both snakes backward
    def handle_back_both_clicked(self):
        if self.control_mode == "Discrete":
            data = 'back_both' # f
            self.pub_.publish(data)
       
    # Continous: both snakes backward     
    def handle_back_both_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "both"
            data = "back_both_cont"
            self.pub_.publish(data)
            
    # Discrete: outer snake forward
    def handle_fwd_outer_clicked(self):
        if self.control_mode == "Discrete":
            data = 'fwd_outer' # x
            self.pub_.publish(data)
    
    # Continous: outer snake forward
    def handle_fwd_outer_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "outer"
            data = "fwd_outer_cont"
            self.pub_.publish(data)
    
    # Discrete: outer snake backward
    def handle_back_outer_clicked(self):
        if self.control_mode == "Discrete":
            data = 'back_outer' # f
            self.pub_.publish(data)
          
    # Continous: outer snake backward
    def handle_back_outer_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "outer"
            data = "back_outer_cont"
            self.pub_.publish(data)
            self.outer_backwards = True
    
    # Discrete: inner snake forward
    def handle_fwd_inner_clicked(self):
        if self.control_mode == "Discrete":
            data = 'fwd_inner' # x
            self.pub_.publish(data)
    
    # Continous inner snake forward
    def handle_fwd_inner_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "inner"
            data = "fwd_inner_cont"
            self.pub_.publish(data)
            self.inner_forwards = True
            
    # Discrete: inner snake backward
    def handle_back_inner_clicked(self):
        if self.control_mode == "Discrete":
            data = 'back_inner' # f
            self.pub_.publish(data)
       
    # Continous: inner snake backward     
    def handle_back_inner_pressed(self):
        if self.control_mode == "Continous":
            self.rail_watch = True
            self.watch_rail = "inner"
            data = "back_inner_cont"
            self.pub_.publish(data)
    
    # Stop all action once buttons released in continous mode
    def continous_mode_button_released(self):
        if self.control_mode == "Continous":
            self.rail_watch = False
            self.watch_rail = None
            data = "stop"
            self.pub_.publish(data) 
            
    # Discrete: tighten outer
    def handle_tighten_outer_clicked(self):
        if self.control_mode == "Discrete":
            data = 'tight_outer' # t
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
        
    # Discrete: loosen outer
    def handle_loosen_outer_clicked(self):
        if self.control_mode == "Discrete":
            data = 'loose_outer' # g
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Discrete: tighten inner
    def handle_tighten_inner_clicked(self):
        if self.control_mode == "Discrete":
            data = 'tight_inner' # v
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Discrete: loosen inner
    def handle_loosen_inner_clicked(self):
        if self.control_mode == "Discrete":
            data = 'loose_inner' # b
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
                
    # Discrete: tighten cable A
    def handle_tighten_outer_A_clicked(self):
        if self.control_mode == "Discrete":
            data = 'tight_outer_A' # i
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Discrete: tighten cable B
    def handle_tighten_outer_B_clicked(self):
        if self.control_mode == "Discrete":
            data = 'tight_outer_B' # k
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
        
    # Discrete: tighten cable C 
    def handle_tighten_outer_C_clicked(self):
        if self.control_mode == "Discrete":
            data = 'tight_outer_C' # j
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Discrete: loosen cable A
    def handle_loosen_outer_A_clicked(self):
        if self.control_mode == "Discrete":
            data = 'loose_outer_A' # p
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
        
    # Discrete: loosen cable B
    def handle_loosen_outer_B_clicked(self):
        if self.control_mode == "Discrete":
            data = 'loose_outer_B' # l  
            self.pub_.publish(data)

    # Discrete: loosen cable C
    def handle_loosen_outer_C_clicked(self):
        if self.control_mode == "Discrete":
            data = 'loose_outer_C' # n
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
                
    # Continous: tighten outer
    def handle_tighten_outer_pressed(self):
        if self.control_mode == "Continous":
            self.tension_watch = True
            self.watch_cable = "outer"
            data = 'outer_tension_cont' # t
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
  
    # Continous: loosen outer      
    def handle_loosen_outer_pressed(self):
        if self.control_mode == "Continous":
            data = 'outer_loosen_cont' # g
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Continous: tighten inner
    def handle_tighten_inner_pressed(self):
        if self.control_mode == "Continous":
            self.tension_watch = True
            self.watch_cable = "inner"
            data = 'inner_tension_cont' # v
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
        
    # Continous: loosen inner
    def handle_loosen_inner_pressed(self):
        if self.control_mode == "Continous":
            data = 'inner_loosen_cont' # b
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
                
    # Continous: tighten A
    def handle_tighten_outer_A_pressed(self):
        if self.control_mode == "Continous":
            self.tension_watch = True
            self.watch_cable = "a"
            data = 'outer_a_tension_cont' # i
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Continous: tighten B
    def handle_tighten_outer_B_pressed(self):
        if self.control_mode == "Continous":
            self.tension_watch = True
            self.watch_cable = "b"
            data = 'outer_b_tension_cont' # k
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Continous: tighten C
    def handle_tighten_outer_C_pressed(self):
        if self.control_mode == "Continous":
            self.tension_watch = True
            self.watch_cable = "c"
            data = 'outer_c_tension_cont' # j
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Continous: loosen A
    def handle_loosen_outer_A_pressed(self):
        if self.control_mode == "Continous":
            data = 'outer_a_loosen_cont' # p
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)
        
    # Continous: loosen B
    def handle_loosen_outer_B_pressed(self):
        if self.control_mode == "Continous":
            data = 'outer_b_loosen_cont' # l  
            self.pub_.publish(data)

    # Continous: loosen C
    def handle_loosen_outer_C_pressed(self):
        if self.control_mode == "Continous":
            data = 'outer_c_loosen_cont' # n
            # rospy.loginfo(chr(data))
            self.pub_.publish(data)

    # Discrete: steer left
    def handle_left_clicked(self):
        data = 'steer_left' # a
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
          
    # Discrete: steer right     
    def handle_right_clicked(self):
        data = 'steer_right' # d
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Discrete: steer up
    def handle_up_clicked(self):
        data = 'steer_up' # y
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    # Discrete: steer down
    def handle_down_clicked(self):
        data = 'steer_down' # h
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    # Discrete & Continous: advance
    def handle_advance_clicked(self):
        self.current_gait = "Advancing"
        data = 'advance' # w
        self.current_gait_compare = self.advance_gait_compare
        self.current_gait_check = [0, 0, 0, 0, 0, 0]
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
       
    # Discrete & Continous: retract   
    def handle_retract_clicked(self):
        self.current_gait = "Retracting"
        data = 'retract' # s
        self.current_gait_compare = self.retract_gait_compare
        self.current_gait_check = [0, 0, 0, 0, 0, 0]
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Discrete & continous: STOP
    def handle_stop_clicked(self):
        data = 'stop' # o
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        self.current_gait = "Ready"
        self.inner_forwards = False
        self.outer_backwards = True
        
    # Discrete & continous: demo
    def handle_demo_clicked(self):
        data = 'demo' # q
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
        
    # Discrete & continous: home snake
    def handle_homing_clicked(self):
        self.current_gait = "Homing"
        self.tension_watch = True
        self.watch_cable = "outer"
        self.current_gait_compare = self.homing_gait_compare
        self.current_gait_check = [0, 0, 0]
        data = 'homing' # ,
        # rospy.loginfo(chr(data))
        self.pub_.publish(data)
    
    # Determine joystick angle
    def determine_joy_angle(self, position_array):
        # Function determines joystick angle by taking x and y positions of joystick, then arctan
        # x position must be first input array element, y position second
        # Returned value is rounded to 1st decimal place
        if position_array[0] == 0 and position_array[1] == 0:
            return 0
        else:
            return round((numpy.arctan2(position_array[1], position_array[0]) * (180/numpy.pi)) + 180, 1)
    

        
    
        