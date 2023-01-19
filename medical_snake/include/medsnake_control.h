/*
 * A ros node that subscribe to user issued command and call medical snake 
 * command functions to move the snake. This node also publishes the smoothed
 * current reading from medsnake dynamixel motors and publish the reading 
 * under the topic /tension_readings
 */

#include <string>
#include <sstream>
#include <ros/ros.h>
#include "std_msgs/Char.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include "medical_snake/Tension_readings.h"
#include "medical_snake.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>


class SnakeControl
{

 public:
  /// Constructor for SnakeControl instance
  SnakeControl(const char* port_name, const char* config_path,
                       const char* dxl_config_path);

  /// initialize the snake
  void snake_initialize(const char* config_path, const char* dxl_config_path);
  
  void command_set(const std_msgs::String::ConstPtr& msg);
  
  void backward_both();

  void forward_both();

  void init_listener();

  void init_tension_publisher();

  void snake_update();

  void publish_tension_reading();

  void publish_snake_mode();

  void emergency_stop();

  void demo();

  void advance();

  void retract();

  void steer_left();

  void steer_right();

  void steer_up();

  void steer_down();

  void tighten_outer();

  void loosen_outer();

  void tighten_inner();

  void loosen_inner();

  void forward_inner();

  void backward_inner();

  void forward_outer();

  void backward_outer();

  void home_rail();

  void home();

  void tighten_outer_A();

  void tighten_outer_B();

  void tighten_outer_C();

  void loosen_outer_A();

  void loosen_outer_B();

  void loosen_outer_C();

  bool cmd_queue_empty(){return command_queue_.empty();};

  std::string get_cmd_queue_top(){return command_queue_[0];};

  bool snake_is_ready(){return snake_.is_ready();};
  bool snake_is_steering() {return snake_.is_steering();};

  void steer_angle();
  void update_steer_angle();

  void joystick_cb(const sensor_msgs::Joy::ConstPtr &msg);

 private:
  MedicalSnake snake_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher mode_pub_;
  ros::NodeHandle nh_;
  ros::Subscriber joystick_sub_;
  std::vector<std::string> command_queue_;
  std::map<std::string, double> tension_dic_;
  float x_joystick_pos, y_joystick_pos;
  bool steering_flag_=false;
};
