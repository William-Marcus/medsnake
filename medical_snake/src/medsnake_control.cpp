#include "medsnake_control.h"

SnakeControl::SnakeControl(const char* port_name, const char* config_path, const char* dxl_config_path) : snake_(port_name)
{
  init_listener();
  init_tension_publisher();
  snake_.initialize(config_path, dxl_config_path);
};

void SnakeControl::command_set(const std_msgs::Char::ConstPtr& msg)
{ // callback function
  ROS_INFO("key pressed: [%c]", char(msg->data));
  char key = msg->data;

  if(key != 'w' && key != 's' && 
     key != 'a' && key != 'd' && 
     key != 'y' && key != 'h' && 
     key != 't' && key != 'g' && 
     key != 'v' && key != 'b' && 
     key != 'e' && key != 'c' && 
     key != 'u' && key != 'm' && 
     key != 'o' && key != 'q' &&
     key != 'i' && key != 'j' && 
     key != 'k' && key != 'p' && 
     key != 'n' && key != 'l' &&
     key != 'r')
  {
    ROS_INFO("[%c] is not a valid command", key);  
  }
    else if (key == 'o') 
  {
    command_queue_.clear();
    command_queue_.push_back(key);
  }   
  else if (snake_.is_ready()) // is command that's not stop and snake is ready
    if (command_queue_.empty()){
      command_queue_.push_back(key);
    }
  else
    ROS_INFO("Last command is executing, invalid key press");
}


void SnakeControl::joystick_cb(const sensor_msgs::Joy::ConstPtr &msg) {
  if (msg->buttons[5] == 1) {
    char key = 'z';
    x_joystick_pos = msg->axes[0];
    y_joystick_pos = msg->axes[1];
    command_queue_.clear();
    command_queue_.push_back(key);
    steering_flag_ = true;
  } else {
    x_joystick_pos = 0;
    y_joystick_pos = 0;
    if (steering_flag_){
      command_queue_.clear();
      steering_flag_ = false;
      ROS_INFO("Press \"Hold\" on the controller to execute steering");
    }
  }
}

void SnakeControl::init_listener()
{ // init subscriber
  sub_ = nh_.subscribe("gui_commands", 100, &SnakeControl::command_set, this);
  joystick_sub_ = nh_.subscribe("joy", 1, &SnakeControl::joystick_cb, this);
}

void SnakeControl::init_tension_publisher()
{
  pub_ = nh_.advertise<medical_snake::Tension_readings>("tension_readings", 1);
  mode_pub_ = nh_.advertise<std_msgs::String>("medsnake_mode", 1);
}

void SnakeControl::snake_update() {snake_.update();}

void SnakeControl::publish_tension_reading()
{
  // publish updated tension readings message
  tension_dic_ = snake_.get_tension_fbk();
  medical_snake::Tension_readings tension_msg;
  tension_msg.header.stamp = ros::Time::now();
  tension_msg.inner_snake_cable = tension_dic_["inner_snake_cable"];
  tension_msg.outer_snake_cable_A = tension_dic_["outer_snake_cable_A"];
  tension_msg.outer_snake_cable_B = tension_dic_["outer_snake_cable_B"];
  tension_msg.outer_snake_cable_C = tension_dic_["outer_snake_cable_C"];

  pub_.publish(tension_msg);
}

void SnakeControl::publish_snake_mode() {
  std_msgs::String mode_msg;
  mode_msg.data = snake_.get_snake_mode();
  mode_pub_.publish(mode_msg);
}

void SnakeControl::emergency_stop()
{
  if (command_queue_.empty()){
    snake_.stop_all_motor();
  }
  else{
    command_queue_.erase(command_queue_.begin());
    snake_.stop_all_motor();
  }
}

void SnakeControl::demo()
{
  command_queue_.erase(command_queue_.begin());

for (int i = 0; i < 1; i++){
  command_queue_.push_back('y'); // Steer Up
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

for (int i = 0; i < 3; i++){
  command_queue_.push_back('y'); // Steer Up
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

for (int i = 0; i < 3; i++){
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

for (int i = 0; i < 3; i++){
  command_queue_.push_back('a'); // 
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

for (int i = 0; i < 3; i++){
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}


for (int i = 0; i < 3; i++){
  command_queue_.push_back('d'); // 
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

for (int i = 0; i < 3; i++){
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}


// for (int i = 0; i < 2; i++)
// {
//   command_queue_.push_back('t'); // Tighten Outer
//   command_queue_.push_back('b'); // Loosen Inner
//   command_queue_.push_back('e'); // Forward Inner
//   command_queue_.push_back('v'); // Tighten Inner
//   command_queue_.push_back('g'); // Loosen Outer
//   command_queue_.push_back('u'); // Forward Outer
// }
// for (int i = 0; i < 4; i++)
// {
//   command_queue_.push_back('y'); // Steer Up
//   command_queue_.push_back('t'); // Tighten Outer
//   command_queue_.push_back('b'); // Loosen Inner
//   command_queue_.push_back('e'); // Forward Inner
//   command_queue_.push_back('v'); // Tighten Inner
//   command_queue_.push_back('g'); // Loosen Outer
//   command_queue_.push_back('u'); // Forward Outer
// }
// for (int i = 0; i < 2; i++)
// {
//   command_queue_.push_back('h'); // Steer Down
//   command_queue_.push_back('t'); // Tighten Outer
//   command_queue_.push_back('b'); // Loosen Inner
//   command_queue_.push_back('e'); // Forward Inner
//   command_queue_.push_back('v'); // Tighten Inner
//   command_queue_.push_back('g'); // Loosen Outer
//   command_queue_.push_back('u'); // Forward Outer
// }
// for (int i = 0; i < 2; i++)
// {
//   command_queue_.push_back('t'); // Tighten Outer
//   command_queue_.push_back('b'); // Loosen Inner
//   command_queue_.push_back('e'); // Forward Inner
//   command_queue_.push_back('v'); // Tighten Inner
//   command_queue_.push_back('g'); // Loosen Outer
//   command_queue_.push_back('u'); // Forward Outer
// }

}

void SnakeControl::advance()
{
  command_queue_.erase(command_queue_.begin());
  // command_queue_.clear();
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

void SnakeControl::retract()
{
  command_queue_.erase(command_queue_.begin());
  // command_queue_.clear();
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('c'); // Backward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('m'); // Backward Outer
}

void SnakeControl::steer_left()
{
  snake_.steer_left();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::steer_right()
{
  snake_.steer_right();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::steer_up()
{
  snake_.steer_up();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::steer_down()
{
  snake_.steer_down();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::tighten_outer()
{
  snake_.tighten_outer();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::loosen_outer()
{
  snake_.loosen_outer();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::tighten_inner()
{
  snake_.tighten_inner();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::loosen_inner()
{
  snake_.loosen_inner();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::forward_inner()
{
  snake_.forward_inner();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::backward_inner()
{
  snake_.backward_inner();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::forward_outer()
{
  snake_.forward_outer();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::backward_outer()
{
  snake_.backward_outer();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::home_rail()
{
  snake_.home_rail();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::tighten_outer_A()
{
  snake_.tighten_outer_A();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::tighten_outer_B()
{
  snake_.tighten_outer_B();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::tighten_outer_C()
{
  snake_.tighten_outer_C();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::loosen_outer_A()
{
  snake_.loosen_outer_A();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::loosen_outer_B()
{
  snake_.loosen_outer_B();
  command_queue_.erase(command_queue_.begin());
}

void SnakeControl::loosen_outer_C()
{
  snake_.loosen_outer_C();
  command_queue_.erase(command_queue_.begin());
}

// Steering to custom angle
void SnakeControl::steer_angle()
{
  snake_.steer_outer(x_joystick_pos,y_joystick_pos);
  // snake_.steer_angle(x_joystick_pos, y_joystick_pos);
  // Want to steer to live angle, so clearing command queue to avoid delay
  // command_queue_.erase(command_queue_.begin());
}

// Steering to custom angle
void SnakeControl::update_steer_angle()
{
  snake_.update_steer_angle_goal(x_joystick_pos,y_joystick_pos);
  // command_queue_.erase(command_queue_.begin());
}

// void SnakeControl::back_to_ready()
// {
//   snake_.back_to_ready();
//   command_queue_.erase(command_queue_.begin());
// }

bool SnakeControl::cmd_queue_empty() {return command_queue_.empty();}

char SnakeControl::get_cmd_queue_top() {return command_queue_[0];}

bool SnakeControl::snake_is_ready() {return snake_.is_ready();}
bool SnakeControl::snake_is_steering() {return snake_.is_steering();}


