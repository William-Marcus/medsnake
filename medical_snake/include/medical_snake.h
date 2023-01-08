/*******************************************************************************
* Copyright Biorobics Lab, Carnegie Mellon University
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Nico Zevallos, Yilin Cai */

/*
                            ___      ___
 .=====.   |        |    | /   \    /   \ |
 |  1  |   |        |    |   3        4   |
 '====='   |        |    | \___/    \___/ |
           |        |    |                |
           |  ___   |    |           ___  |
 .=====.   | /   \  |    |          /   \ |
 |  0  |   |   2    |    |            5   |
 '====='   | \___/  |    |          \___/ |
 
             \-----5---------4 
              \     \       /    
         0 ----\-----\--2  /    
                1     \   /     
                 \     \ / 
                  \-----3   
*/


#ifndef MEDICAL_SNAKE_H
#define MEDICAL_SNAKE_H

#include "dynamixel_controller.h"
#include <yaml-cpp/yaml.h>

enum modes { 
  CONNECTION_ERROR,
  HARDWARE_ERROR,
  UNINITIALIZED,
  INITIALIZING,
  READY,
  MOVING_POSITION, // for all commands that uses position control: 
  // loosen(inner/outer), steering, forward(inner/outer), backward(inner/outer)
  TIGHTENING, // for commands that uses force control: tighen(inner/outer)
  TIGHTENING_INNER,
  TIGHTENING_OUTER,
  STEERING,
  HOMING_RAIL,
};

class MedicalSnake : protected DynamixelController
{
 public:
  /**
   * Create a MedicalSnake instance
   * 
   * @param port_name Medical snake USB port (Linux /dev/ttyUSB[X] Windows COM[X])
   */
  MedicalSnake(const char* port_name);

  /**
   * Initialize MedicalSnake
   * 
   * @param config_path Path to medical snake configuration ([PATH_TO_SRC]/config/medical_snake.yaml)
   * @param dxl_config_path Path to dynamixel configuration ([PATH_TO_SRC]/config/dynamixels.yaml)
   */
  void initialize(const char* config_path, const char* dxl_config_path);

  /// Uninitialize DynamixelController
  bool uninitialize() { return DynamixelController::uninitialize(); };

  void get_max_position_limit_and_init_smooth_current();

  /// Check whether inner snake rail limit switch is pressed
  bool inner_limit_switch();

  /// Check whether outer snake rail limit switch is pressed
  bool outer_limit_switch();

  /// Tighten inner snake
  void tighten_inner();

  /// Tighten outer snake
  void tighten_outer();

  /// Tighten individual outer snake cable
  void tighten_outer_A();
  void tighten_outer_B();
  void tighten_outer_C();

  /// Loose inner snake
  void loosen_inner();

  /// Loose outer snake
  void loosen_outer();

    /// Loose individual outer snake cable
  void loosen_outer_A();
  void loosen_outer_B();
  void loosen_outer_C();

  /// Forward inner snake
  void forward_inner();

   /// Forward outer snake
  void forward_outer();

  /// Backward inner snake
  void backward_inner();

  /// Backward outer snake
  void backward_outer();

  /// Steer left
  void steer_left();

  /// Steer right
  void steer_right();

  /// Steer up
  void steer_up();

  /// Steer down
  void steer_down();

  /// Homing rail
  void home_rail();

  /// Perform goal checking for current executing command function and 
  /// write register while the goal is not reached
  void update();

  /// check if the goal set by command function is reached, return a map of 
  /// whether the goal is reached for each Dynamixel
  std::map<std::string, bool> check_goal();

  /// stop all moving motor
  void stop_moving_motor();

  void stop_all_motor();

  /**
   * Get the ideal speed factor when the snake is tightening
   * 
   * @param motor_names a vector of Dynamixel name as defined in YAML file
   * @param is_tighten whether the tendon is tight
   */
  std::map<std::string, float> get_ideal_speeds(std::vector<std::string> motor_names,
                                                std::map<std::string, bool> is_tight);

  /**
   * Steer to angle
   * 
   * @param y_joystick_pos the float y position of the joystick
   * @param x_joystick_pos the float x position of the joystick
   */
  void steer_angle(float x_joystick_pos, float y_joystick_pos);


  /**
   * Steer to angle
   * 
   * @param y_joystick_pos the float y position of the joystick
   * @param x_joystick_pos the float x position of the joystick
   */
  void steer_outer(float x_joystick_pos, float y_joystick_pos);
  void update_steer_angle_goal(float x_joystick_pos, float y_joystick_pos);


  /**
   * Move every motor in motor_names by a common specified radian
   * 
   * @param motor_names a vector of Dynamixel name as defined in YAML file
   * @param radians radian to move for each motor
   */
  void move_position(const std::vector<std::string> motor_names, float radians);

  void move_position_adjust_velocity(std::map<std::string, float> motor_and_radian);

  /**
   * Move each motor in motor_and_radian by the specified radian
   * 
   * @param motor_and_radian a map of Dynamixel name as defined in YAML file to 
   * the corresponding radian by which it should move
   */
  void move_position(std::map<std::string, float> motor_and_radian);

  /**
   * Stop every motor in names
   * 
   * @param names a vector of Dynamixel name as defined in YAML file
   */
  void stop_motor(const std::vector<std::string> names);

  /**
   * Convert the given radian to the corresponding value to write to the Dynamixel
   * 
   * @param motor_name the name of the Dynamixel for which we need the conversion
   * @param radians the value of radian to convert
   */
  float radian_to_value(const std::string motor_name, const float radians);

  /**
   * Set medsnake to new mode and perform safety check before the new mode is set
   * 
   * @param new_mode the new mode to set
   */
  void set_mode(modes new_mode);
  
  /**
   * Set the operation mode of the passed in Dynamixel names
   * 
   * @param opmode the operation mode to be set
   * @param motor_names the name of the Dynamixel for which we need to set the operation mode
   */
  bool set_opmode(const int32_t opmode, const std::vector<std::string> motor_names);

  /**
   * Set the profile velocity of the passed in Dynamixel names
   * assumes the Dynamxiels are under extended position control mode
   * 
   * @param motor_names the name of the Dynamixel for which we need to set the operation mode
   * @param profile_velocity the motor velocity by which each motor should move
   */
  bool set_profile_velocity(const std::vector<std::string> motor_names, 
                            std::vector<int32_t> profile_velocity);

  /**
   * Write registers on multiple dynamixels simultaneously
   * 
   * @param[in] names Dynamixel names as defined in YAML file
   * @param[in] item_name Name of parameter as defined in control table
   * @param[out] data Values to write to register on each dynamixel
   * \return True if registers were written to, false if not
   */
  bool sync_write_register(const std::vector<std::string> names,
                           const std::string item_name,
                           std::vector<int32_t> data) {
    return DynamixelController::sync_write_register(names, item_name, data);
  };

  /**
   * Read Get parameter value from last feedback received during update
   * 
   * @param name Dynamixel name as defined in YAML file
   * @param item_name Name of parameter as defined in control table
   * \return Feedback value address
   */
  int32_t get_fbk(const std::string dxl_name, const std::string item_name) {
    return DynamixelController::get_fbk(dxl_name, item_name);
  };

  /// a temporary debug function
  void print_goal();

  /// return whether the snake in in mode ready
  bool is_ready() {
    return medsnake_mode_ == modes::READY;
  }

  /// return whether the snake in in mode steering
  bool is_steering() {
    return medsnake_mode_ == modes::STEERING;
  }

  /// return a map from cable names to their tension measure by current 
  /// TODO: change to force sensor value
  std::map<std::string, double> get_tension_fbk();

  /// return the snake mode in form of string
  std::string get_snake_mode();

  double tension_reading(std::string motor_name);



 private:

  /// a base velocity that each motor will not exceed, this ensure large delta
  /// in position change can be properly handled
  int32_t max_velocity_; 

  ///
  int32_t feeding_velocity_;

  /// The constant velocity for velocity control
  int32_t goal_velocity_;

  int32_t goal_tension_outer_;
  int32_t goal_tension_inner_;

  /// Introduced a timestamp to fix the peak current issue when the motor starts moving
  /// no longer needed with tension sensing
  // std::chrono::time_point<std::chrono::high_resolution_clock> tighten_command_timestamp_;

  /// Introduced a bool value to record first loop of update to fix the peak 
  /// current issue when the motor starts moving, no longer needed with tension sensing
  // bool tighten_first_pass_ = true;

  /// medsnake mode default to UNINITIALIZED
  modes medsnake_mode_ = UNINITIALIZED;
  
  /// the goals set by the executing command function
  std::map<std::string, int32_t> goals_;

  /// the max position limit for every motor
  std::map<std::string, int32_t> max_position_limit_;

  /// the current offsets for each motor
  std::map<std::string, int32_t> current_offset_;

  /// smoothed current
  std::map<std::string, int32_t> smooth_current_;

  std::map<std::string, double> calib_coeff_;
  std::map<std::string, double> calib_offset_;

  std::map<std::string, int32_t> present_steer_center_outer_;

  // float steer_outer_x_;
  // float steer_outer_y_;
};

#endif //MEDICAL_SNAKE_H