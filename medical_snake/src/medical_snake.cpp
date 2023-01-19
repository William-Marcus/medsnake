#include <iostream>
#include "medical_snake.h"
#include <assert.h>
#include <math.h>

#define CURRENT_CONTROL_MODE 0
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define EXTENDED_POSITION_CONTROL_MODE 4
#define PWM_CONTROL_MODE 16

const std::vector<std::string> outer_snake_cable = {"outer_snake_cable_A", 
                                                    "outer_snake_cable_B", 
                                                    "outer_snake_cable_C"};


MedicalSnake::MedicalSnake(const char* port_name)
 : DynamixelController(port_name)
{

};

void MedicalSnake::initialize(const char* config_path, const char* dxl_config_path)
{
  if(!config_path || !dxl_config_path)
    throw std::runtime_error("[MedicalSnake::initialize] Configuration paths cannot be null");
  YAML::Node config;
  config = YAML::LoadFile(config_path);

  if (config.IsNull())
    throw std::runtime_error("[MedicalSnake::initialize] Medical snake configuration file is missing or badly formed");

  // Basic feedback for normal medsnake function
  std::vector<std::string> feedback_items = {
    {"Moving"                },
    {"Present_Current"       },
    {"Present_Velocity"      },
    {"Present_Position"      },
    {"External_Port_Data_1"  },
    {"External_Port_Data_2"  },
    {"Hardware_Error_Status" },
    // Commandable keys
    {"Torque_Enable"        },
    {"Bus_Watchdog"         },
    {"Goal_Current"         },
    {"Goal_Velocity"        },
    {"Goal_Position"        },
    {"Bus_Watchdog"         },
  };

  bool has_n_links = false;
  bool has_link_length = false;
  bool has_rail_screw_lead = false;
  bool has_max_cable_velocity = false;
  bool has_goal_velocity = false;
  bool has_feeding_velocity = false;
  bool has_goal_tension_inner = false;
  bool has_goal_tension_outer = false;


  for (auto it_file = config.begin(); it_file != config.end(); ++it_file)
  {
    // Category
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0) continue;
    if (name == "n_links") has_n_links = true;
    if (name == "link_length") has_link_length = true;
    if (name == "rail_screw_lead") has_rail_screw_lead = true;
    if (name == "max_cable_velocity"){
      max_cable_velocity_ = it_file->second.as<int32_t>();
      has_max_cable_velocity = true;
    }
    if (name == "feeding_velocity"){
      feeding_velocity_ = it_file->second.as<int32_t>();
      has_feeding_velocity = true;
    }
    if (name == "goal_velocity"){
      goal_velocity_ = it_file->second.as<int32_t>();
      has_goal_velocity = true;
    }
    if (name == "goal_tension_outer"){
      goal_tension_outer_ = it_file->second.as<int32_t>();
      has_goal_tension_outer = true;
    }
    if (name == "goal_tension_inner"){
      goal_tension_inner_ = it_file->second.as<int32_t>();
      has_goal_tension_inner = true;
    }
    if (name == "feedback_items"){
      auto extra_items = it_file->second.as<std::vector<std::string>>();
      feedback_items.insert(feedback_items.end(), extra_items.begin(),
                            extra_items.end());
    }
    if (name == "current_offset")
    {
      YAML::Node item = config[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        current_offset_[it_item->first.as<std::string>()] = it_item->second.as<float>();
      }
    }
    if (name == "calibration_coefficient")
    {
      YAML::Node item = config[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        calib_coeff_[it_item->first.as<std::string>()] = it_item->second.as<float>();
      }
    }
    if (name == "calibration_offset")
    {
      YAML::Node item = config[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        calib_offset_[it_item->first.as<std::string>()] = it_item->second.as<float>();
      }
    }
  }
  assert(has_n_links == true && has_link_length == true && 
         has_rail_screw_lead == true && has_max_cable_velocity == true);
  // Check for necessary variables
  if (!DynamixelController::initialize(dxl_config_path, feedback_items))
  {
    throw std::runtime_error("[MedicalSnake::initialize] Medical snake failed to initialize");
  }
  // initialize the map for max position limit and init smooth current
  get_max_position_limit_and_init_smooth_current();

  // set to ready manually when snake successfully initializes
  medsnake_mode_ = modes::READY;
};


double MedicalSnake::tension_reading(std::string motor_name)
{
  double result;
  int32_t raw = get_fbk(motor_name, "External_Port_Data_1");
  result = double(raw) - (calib_offset_[motor_name] * 4096 / 3.3);
  result = result * 3.3 / 4096;
  result = result * calib_coeff_[motor_name];
  result = 4.448 * result; //lbs to N
  return result;
}

void MedicalSnake::get_max_position_limit_and_init_smooth_current()
{
  for(const char *motor_name : {"outer_snake_cable_A", "outer_snake_cable_B", 
                               "outer_snake_cable_C", "inner_snake_cable", 
                               "inner_snake_rail", "outer_snake_rail"})
  {
    int32_t max_pos;
    read_register(motor_name, "Max_Position_Limit", &max_pos);
    max_position_limit_[motor_name] = max_pos;
    smooth_current_[motor_name] = 0.0;
  }
}

// tentative code adapted from medsnake old code starts
float MedicalSnake::radian_to_value(const std::string motor_name,
                                    const float radians)
{
  int32_t max_pos = max_position_limit_[motor_name];
  return radians * max_pos / 3.14159265359;
};

// pass in a raw reading, return the number in decimal as if the 
// readings a twos complement number
int32_t twos_comp(const int32_t reading, const int size)
{
  if (reading > (1 << (size - 1))) return reading - (1 << size);
  else return reading;
}

bool approx_equal(const int32_t a, const int32_t b)
{
  if (abs(a - b) < 2){
    return true;
  }
  return false;
}

template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

std::map<std::string, bool> MedicalSnake::check_goal()
{
  
  std::map<std::string, bool> goal_reached;
  float smoothing = 5.0;
  for (const std::pair<const std::string, int32_t> &goal : goals_)
  {
    goal_reached[goal.first] = false;
  }
  // if (goals.empty()) return goal_reached; // will return if the goal is empty
  // for homing goal should be in form {"inner_snake_rail": 1, "outer_snake_rail": 1}
  switch (medsnake_mode_)
  {
  case modes::HOMING_RAIL:  
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = (get_fbk(goal.first, "External_Port_Data_2") == goal.second);
    }
    break;
// -------------------------------------------------------------------------------------------------------------------
  case modes::TIGHTENING:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = tension_reading(goal.first) >= goal.second;
      std::cout << "**** Goal for motor " << goal.first << " reached? " << goal_reached[goal.first]<< "\n";      
    }
    std::cout << "**** \n";
    break;

  case modes::TIGHTENING_OUTER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = tension_reading(goal.first) >= goal.second;
      print_goal();
      std::cout << "tension_reading(goal.first)" << tension_reading(goal.first) << "\n";
      std::cout << "**** Goal for motor " << goal.first << " reached? " << goal_reached[goal.first]<< "\n";      
    }
    std::cout << "**** \n";
    break;
  case modes::TIGHTENING_INNER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = tension_reading(goal.first) >= goal.second;
      std::cout << "**** Goal for motor " << goal.first << " reached? " << goal_reached[goal.first]<< "\n";      
    }
    std::cout << "**** \n";
    break;

// --------------------------------------------------------------------------------------------------------------------
  case modes::MOVING_POSITION:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::MOVING_BACKWARD:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::MOVING_FORWARD:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::MOVING_OUTER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::MOVING_INNER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::STEERING:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::DEMO:
    // for (const std::pair<const std::string, int32_t> &goal : goals_)
    // {
    //   goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
    //   // alternatively, can use "moving" for goal_reached
    // }
    break;
  case modes::TIGHTENING_CABLE:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::LOOSENING_CABLE:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::LOOSENING_OUTER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  case modes::LOOSENING_INNER:
    for (const std::pair<const std::string, int32_t> &goal : goals_)
    {
      goal_reached[goal.first] = approx_equal(get_fbk(goal.first, "Present_Position"), goal.second);
      // alternatively, can use "moving" for goal_reached
    }
    break;
  default:
    throw std::runtime_error("[MedicalSnake::check_goal] Medical snake has no goal under this command mode");
  }
  return goal_reached;
}

void MedicalSnake::set_mode(modes new_mode)
{
  print_goal();
  // medical snake is in proper mode to reset the goals
  if (medsnake_mode_ != UNINITIALIZED && medsnake_mode_ != CONNECTION_ERROR && 
      medsnake_mode_ != HARDWARE_ERROR)
  {
    goals_.clear(); // clear the goal vector
    medsnake_mode_ = new_mode;
  }
  else
  {
    throw std::runtime_error("[MedicalSnake::set_mode] Medical snake unable to change mode in current mode");
  }
}


void MedicalSnake::update()
{
  // update to register
  DynamixelController::update();
  // container for names of motor that have not reached its goal or need to write to stop
  std::vector<std::string> goal_to_write;
  // container for goal of motor that have not reached its goal
  std::vector<int32_t> goal_quantity; 
  // container for names of motor to stop
  std::vector<std::string> goal_to_stop;

  // only effective if velocity control mode
  // int32_t goal_velocity_ = 250;
  bool all_stop = true;
  std::vector<std::string> all_motor_names;
  std::map<std::string, bool> checked_goal;
  std::map<std::string, float> ideal_speed;
  // if goal is reached (medsnake_mode_ set to READY)
  if (medsnake_mode_ == modes::READY)
  {
    std::cout << "**************snake ready for commands**************\n";
    return;
  }
  checked_goal = check_goal();
  if (medsnake_mode_ == modes::TIGHTENING || medsnake_mode_ == modes::TIGHTENING_OUTER || medsnake_mode_ == modes::TIGHTENING_INNER) // modify get ideal speed based on tension reading
  {
    for (const std::pair<const std::string, bool> &motor_pair : checked_goal)
    {
      all_motor_names.push_back(motor_pair.first);
    }
    ideal_speed = get_ideal_speeds(all_motor_names, checked_goal);
  }

  for (const std::pair<const std::string, bool> &motor_pair : checked_goal)
  {
    if (!motor_pair.second)
    { // if the goal is not reached for this motor
      goal_to_write.push_back(motor_pair.first);
      
      if (medsnake_mode_ == modes::MOVING_POSITION || medsnake_mode_ == modes::MOVING_BACKWARD || medsnake_mode_ == modes::MOVING_FORWARD || medsnake_mode_ == modes::MOVING_INNER
      || medsnake_mode_ == modes::MOVING_OUTER || medsnake_mode_ == modes::TIGHTENING_CABLE ||
       medsnake_mode_ == modes::LOOSENING_CABLE || medsnake_mode_ == modes::LOOSENING_OUTER || medsnake_mode_ == modes::LOOSENING_INNER)
      { // if (extended) position control mode
      // push back goals directly 
        goal_quantity.push_back(goals_[motor_pair.first]);
      }
      else if (medsnake_mode_ == STEERING)
      {  
        goal_quantity.push_back(goals_[motor_pair.first]);
        // std::cout << "Steering outer, goal position for :" << motor_pair.first<<" is"<< goals_[motor_pair.first] << "\n";
        all_stop = false;
      }
      // if velocity control mode
      else if (medsnake_mode_ == HOMING_RAIL)
      { 
        goal_quantity.push_back(feeding_velocity_);
        all_stop = false;
      }
      else if (medsnake_mode_ == modes::TIGHTENING || medsnake_mode_ == modes::TIGHTENING_OUTER || 
      medsnake_mode_ == modes::TIGHTENING_INNER) 
      {
        goal_quantity.push_back(int32_t(-goal_velocity_ * ideal_speed[motor_pair.first]));
        all_stop = false;
      }

    }
    else
    { // if a single motor has reached its goal

      if (medsnake_mode_ == HOMING_RAIL || medsnake_mode_ == TIGHTENING ||
          medsnake_mode_ == TIGHTENING_OUTER || medsnake_mode_ == TIGHTENING_INNER)
      { // if under velocity control mode
        // push 0 goal velocity to stop motor
        goal_to_write.push_back(motor_pair.first);
        goal_quantity.push_back(0);
      }
      
    }
  }


  if (medsnake_mode_ == modes::HOMING_RAIL)
  {
    if (!all_stop)
    { // if the goal is not reached for all motor
      sync_write_register(goal_to_write, "Goal_Velocity", goal_quantity);
    }
    else
    { // if the goal is reached for all motors
      // write 0 to stop
      sync_write_register(goal_to_write, "Goal_Velocity", goal_quantity);
      set_mode(modes::READY);
      std::cout << "---------------homing goal is reached---------------\n";
    }
  }
  else if (medsnake_mode_ == modes::TIGHTENING || medsnake_mode_ == modes::TIGHTENING_OUTER || 
      medsnake_mode_ == modes::TIGHTENING_INNER)
  {
    // if the goal is not reached for all motor or the function has not exectued for more than 2 cycles (7 Hz)
    if (!all_stop)
    { 
      // std::cout << duration.count() << " ms~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
      // std::vector<std::int32_t> vel_goal(goal_to_write.size(), -goal_velocity_);
      sync_write_register(goal_to_write, "Goal_Velocity", goal_quantity);
    }
    else
    { // if the goal is reached for all motors
      // write 0 to stop
      sync_write_register(goal_to_write, "Goal_Velocity", goal_quantity);
      set_mode(modes::READY);
      std::cout << "---------------tighten goal is reached------------------\n";
    }
  }
  else if (medsnake_mode_ == modes::MOVING_POSITION || medsnake_mode_ == modes::MOVING_BACKWARD || medsnake_mode_ == modes::MOVING_FORWARD 
      || medsnake_mode_ == modes::MOVING_INNER || medsnake_mode_ == modes::MOVING_OUTER 
      || medsnake_mode_ == modes::TIGHTENING_CABLE || medsnake_mode_ == modes::LOOSENING_CABLE 
      || medsnake_mode_ == modes::LOOSENING_OUTER || medsnake_mode_ == modes::LOOSENING_INNER)
  { // assuming position problem is fixed
    if (!goal_to_write.empty())
    { // if the goal is not reached for all motor
      sync_write_register(goal_to_write, "Goal_Position", goal_quantity);
    }
    else
    { // if the goal is reached for all motors
      set_mode(modes::READY);
      std::cout << "---------------position goal is reached------------------\n";
    }
  }
  else if (medsnake_mode_ == modes::STEERING)
  { 
    if (!goal_to_write.empty())
    { // if the goal is not reached for all motor
      sync_write_register(goal_to_write, "Goal_Position", goal_quantity);
    }
    // We will never set the goal reached in STEERING
    // We wiil exit STEERING to READY by calling emergency_stop() in medical_snake_ros.cpp

  }

}



void MedicalSnake::stop_all_motor() {
  stop_motor({"outer_snake_cable_A", "outer_snake_cable_B", 
              "outer_snake_cable_C", "inner_snake_cable", 
              "inner_snake_rail", "outer_snake_rail"});
  set_mode(modes::READY);
}

void MedicalSnake::stop_motor(const std::vector<std::string> names)
{
  set_opmode(VELOCITY_CONTROL_MODE, names);
  std::vector<int> stop(names.size(), 0);
  if(!DynamixelController::sync_write_register(names, "Goal_Velocity", stop))
  { // set goal velocity to 0
    throw std::runtime_error("[MedicalSnake::stop_motor] Medical snake failed to set goal velocity to 0");
  } 
}

std::map<std::string, float> MedicalSnake::get_ideal_speeds(std::vector<std::string> motor_names,
                                                            std::map<std::string, bool> is_tight)
{
  float min_tension = std::numeric_limits<float>::max();
  float max_tension = std::numeric_limits<float>::min();
  // Find minimum and maximum currents
  std::map<std::string, float> result;
  for (auto const& motor:motor_names)
  {
    if(is_tight[motor]) continue;
    if(abs(smooth_current_[motor]) < min_tension) min_tension = abs(smooth_current_[motor]);
    if(abs(smooth_current_[motor]) > max_tension) max_tension = abs(smooth_current_[motor]);
  }
  // Find speed for each motor
  float max_speed = 0.0f;
  for (auto const& motor:motor_names)
  {
    if(is_tight[motor]) continue;
    float diff = max_tension - abs(smooth_current_[motor]);
    diff = (100.0f > diff) ? diff : 100;
    result[motor] = 1.0f - (diff / 100.0f);
    if(result[motor] > max_speed)
    {
      max_speed = result[motor];
    }
  }
  // Normalize speeds
  for (auto const& motor:motor_names)
  {
    if(is_tight[motor])
    {
      result[motor] = 0;
      continue;
    }
    result[motor] += 1.0f - max_speed; //result is actually scale to max speed
  }
  return result;
}


bool MedicalSnake::set_opmode(const int32_t opmode, const std::vector<std::string> motor_names)
{
  bool need_set_opmode = false;
  for (std::string name : motor_names)
  {
    int32_t op_mode;
    MedicalSnake::read_register(name, "Operating_Mode", &op_mode);
    if(op_mode != opmode) need_set_opmode = true;

  }
  if(need_set_opmode) 
  {
    std::vector<int32_t> torque_off(motor_names.size(), 0);
    if(!DynamixelController::sync_write_register(motor_names, "Torque_Enable", torque_off))
    { // set torque off
      throw std::runtime_error("[MedicalSnake::set_opmode] Medical snake failed to set torque off");
      return false;
    }
    std::vector<int32_t> operation_mode(motor_names.size(), opmode);
    // set all motors to extended position control mode
    if(!DynamixelController::sync_write_register(motor_names, "Operating_Mode", operation_mode))
    {
      throw std::runtime_error("[MedicalSnake::set_opmode] Medical snake failed to set to extended position control mode");
      return false;
    }
    std::vector<int32_t> torque_on(motor_names.size(), 1);
    if(!DynamixelController::sync_write_register(motor_names, "Torque_Enable", torque_on))
    { // set torque on
      throw std::runtime_error("[MedicalSnake::set_opmode] Medical snake failed to set torque on");
      return false;
    }
  }
  else {return true;}
  
  return true;
}

bool MedicalSnake::set_profile_velocity(const std::vector<std::string> motor_names,
                                        std::vector<int32_t> profile_velocity)
{
  if(!DynamixelController::sync_write_register(motor_names, "Profile_Velocity", profile_velocity))
  { // set torque off
    throw std::runtime_error("[MedicalSnake::set_profile_velocity] Medical snake failed to set profile velocity");
    return false;
  }
  return true;
}

// vector version, every motor moves by same delta in radian
void MedicalSnake::move_position(const std::vector<std::string> motor_names, const float radians)
{ 
  set_opmode(EXTENDED_POSITION_CONTROL_MODE, motor_names);
  // all motor moves by same radian thus all max_cable_velocity_
  if(motor_names.size() > 1 || (motor_names[0] == "outer_snake_cable_A" ||
                                motor_names[0] == "outer_snake_cable_B" ||
                                motor_names[0] == "outer_snake_cable_C" ))
  { // except for railing, profile velocity is set 
    std::vector<int32_t> profile_vel(motor_names.size(), max_cable_velocity_);
    set_profile_velocity(motor_names, profile_vel);
  }

  if( (motor_names.size() == 1 || motor_names.size() == 2)
   && (motor_names[0] == "outer_snake_rail" || motor_names[0] == "inner_snake_rail" ) )
  { // set profile velocity for rail
    std::vector<int32_t> profile_vel(motor_names.size(), feeding_velocity_);
    set_profile_velocity(motor_names, profile_vel);
  }

  //   set goals
  for (const std::string &name : motor_names)
  {
    int32_t present_position = get_fbk(name, "Present_Position");
    goals_[name] = present_position + int32_t(radian_to_value(name, radians));
  }
}

// map version, each motor takes on different delta in radian
void MedicalSnake::move_position(std::map<std::string, float> motor_and_radian)
{
  std::vector<std::string> motor_names;
  for(const std::pair<const std::string, float> &pair : motor_and_radian){
    motor_names.push_back(pair.first);
  }
  set_opmode(EXTENDED_POSITION_CONTROL_MODE, motor_names);

  move_position_adjust_velocity(motor_and_radian);
  
  for(const std::pair<const std::string, float> &pair : motor_and_radian)
  {
    int32_t present_position = get_fbk(pair.first, "Present_Position");
    goals_[pair.first] = present_position + int32_t(radian_to_value(pair.first, pair.second));
  }
}


void MedicalSnake::move_position_adjust_velocity(std::map<std::string, float> motor_and_radian)
{
  std::vector<std::string> motor_names;
  std::vector<int32_t> adjusted_velocity;

  float max_delta = abs(motor_and_radian.begin()->second); // init max to the first element
  std::string max_motor_name = motor_and_radian.begin()->first;

  for(const std::pair<const std::string, float> &pair : motor_and_radian)
  {
    motor_names.push_back(pair.first);
    if (abs(pair.second) > max_delta)
    {
      max_delta = abs(pair.second);
      max_motor_name = pair.first;
    }
  }
  float factor = max_cable_velocity_ / motor_and_radian[max_motor_name];
  for(const std::pair<const std::string, float> &pair : motor_and_radian)
  {
    // std::cout <<"!!!!!!!"<< pair.first<< " speed: "<< int32_t(abs(pair.second * factor))<<"\n";
    adjusted_velocity.push_back(int32_t(abs(pair.second * factor)));
  }
  set_profile_velocity(motor_names, adjusted_velocity);
}





// ================ tigenten outer/inner based on tendon tension ===============

void MedicalSnake::tighten_outer()
{
  // tighten_command_timestamp_ = std::chrono::high_resolution_clock::now();
  set_mode(modes::TIGHTENING_OUTER);
  set_opmode(VELOCITY_CONTROL_MODE, outer_snake_cable);
  goals_ = {{"outer_snake_cable_A", goal_tension_outer_}, {"outer_snake_cable_B", goal_tension_outer_}, 
            {"outer_snake_cable_C", goal_tension_outer_}}; 
}

void MedicalSnake::tighten_inner()
{
  // tighten_command_timestamp_ = std::chrono::high_resolution_clock::now();
  set_mode(modes::TIGHTENING_INNER);
  set_opmode(VELOCITY_CONTROL_MODE, {"inner_snake_cable"});
  // current goal (force goal)
  goals_ = {{"inner_snake_cable", goal_tension_inner_}};
}
// =============================================================================




// ================ tigenten/loosen outer/inner based on position ===============

void MedicalSnake::tighten_outer_A()
{
//   tighten_command_timestamp_ = std::chrono::high_resolution_clock::now();
//   tighten_first_pass_ = true;
//   set_mode(modes::TIGHTENING);
//   set_opmode(VELOCITY_CONTROL_MODE, {"outer_snake_cable_A"});
//   goals_ = {{"outer_snake_cable_A", 130}};
  set_mode(modes::TIGHTENING_CABLE);
  float radians = -0.5;
  move_position({"outer_snake_cable_A"}, radians);
}

void MedicalSnake::tighten_outer_B()
{
  // tighten_command_timestamp_ = std::chrono::high_resolution_clock::now();
  // tighten_first_pass_ = true;
  // set_mode(modes::TIGHTENING);
  // set_opmode(VELOCITY_CONTROL_MODE, {"outer_snake_cable_B"});
  // goals_ = {{"outer_snake_cable_B", 130}};
  set_mode(modes::TIGHTENING_CABLE);
  float radians = -0.5;
  move_position({"outer_snake_cable_B"}, radians);
}

void MedicalSnake::tighten_outer_C()
{
  // tighten_command_timestamp_ = std::chrono::high_resolution_clock::now();
  // tighten_first_pass_ = true;
  // set_mode(modes::TIGHTENING);
  // set_opmode(VELOCITY_CONTROL_MODE, {"outer_snake_cable_C"});
  // goals_ = {{"outer_snake_cable_C", 130}};
  set_mode(modes::TIGHTENING_CABLE);
  float radians = -0.5;
  move_position({"outer_snake_cable_C"}, radians);
}

void MedicalSnake::loosen_outer()
{
  set_mode(modes::LOOSENING_OUTER);
  float radians = 0.4;
  move_position(outer_snake_cable, radians);
}

// loosen individual outer snake cable
void MedicalSnake::loosen_outer_A()
{
  set_mode(modes::LOOSENING_CABLE);
  float radians = 0.5;
  move_position({"outer_snake_cable_A"}, radians);
}

void MedicalSnake::loosen_outer_B()
{
  set_mode(modes::LOOSENING_CABLE);
  float radians = 0.5;
  move_position({"outer_snake_cable_B"}, radians);
}

void MedicalSnake::loosen_outer_C()
{
  set_mode(modes::LOOSENING_CABLE);
  float radians = 0.5;
  move_position({"outer_snake_cable_C"}, radians);
}

void MedicalSnake::loosen_inner()
{
  set_mode(modes::LOOSENING_INNER);
  float radians = 0.4; // parameter of turn
  move_position({"inner_snake_cable"}, radians);
  print_goal();
}
// =============================================================================





//================ Forward / backword outer or inner snake =====================

void MedicalSnake::forward_inner()
{
  set_mode(modes::MOVING_INNER);
  float radians = -6.4; // 3.2 is for forwarding 1 link length
  // TODO(Maggie): Change the value by experiment if necessary
  move_position({"inner_snake_rail"}, radians);
  print_goal();
}


void MedicalSnake::forward_outer()
{
  set_mode(modes::MOVING_OUTER);
  float radians = -6.4; // 3.2 is for forwarding 1 link length
  // TODO(Maggie): Change the value by experiment if necessary
  move_position({"outer_snake_rail"}, radians);
}


void MedicalSnake::backward_inner()
{
  set_mode(modes::MOVING_INNER);
  float radians = 6.4; // 3.2 is for forwarding 1 link length
  // TODO(Maggie): Change the value by experiment if necessary
  move_position({"inner_snake_rail"}, radians);
}


void MedicalSnake::backward_outer()
{
  set_mode(modes::MOVING_OUTER);
  float radians = 6.4; // 3.2 is for forwarding 1 link length
  // TODO(Maggie): Change the value by experiment if necessary
  move_position({"outer_snake_rail"}, radians);
}

void MedicalSnake::backward_both()
{
  set_mode(modes::MOVING_BACKWARD);
  float radians = 6.4;
  move_position({"inner_snake_rail", "outer_snake_rail"}, radians);
  print_goal();
}

void MedicalSnake::forward_both()
{
  set_mode(modes::MOVING_FORWARD);
  float radians = -6.4;
  move_position({"outer_snake_rail", "inner_snake_rail"}, radians);
  print_goal();
}
// ===========================================================================






//================ Steering the outer snake continuously =====================

void MedicalSnake::steer_angle(float x, float y) {
  set_mode(modes::STEERING);

  x = -x;
  
  double CR = .0047;
  // Finding phi in radians
  float phi = sqrt(pow(x, 2) + pow(y, 2)) * .25 * M_PI;

  if (phi > (M_PI / 6)) {
    phi = M_PI / 6;
  } 

  // Finding theta in radians
  float theta = atan2(y, x) - M_PI*.5;
  float L = .0085;
  float r = .0047;

  float A = (sqrt(2*pow(r, 2)*pow(cos(theta), 2)*(1-cos(phi)) + 2 * L * r * cos(theta) * sin(phi) + pow(L, 2)) - L) / .01;

  float B = (sqrt(pow((r/2 - r/2 * (cos(phi) * pow(cos(theta), 2) + pow(sin(theta), 2)) - L * cos(theta) * sin(phi) + sqrt(3) / 2 * r * (cos(theta) * sin(theta) * (1 - cos(phi)))) , 2)    
            + pow((r/2 * cos(theta) * sin(phi) - L * cos(phi) + sqrt(3) / 2 * r * sin(phi) * sin(theta)), 2) 
            + pow(sqrt(3) / 2 * r + r / 2 * (cos(theta) * sin(theta) * (1 - cos(phi))) - sqrt(3) / 2 * r * (pow(cos(theta), 2) + cos(phi) * pow(sin(theta), 2)) - L * sin(phi) * sin(theta) , 2)) - L) / .01;

  float C = (sqrt(pow((-r/2 + r/2 * (cos(phi) * pow(cos(theta), 2) + pow(sin(theta), 2)) + L * cos(theta) * sin(phi) + sqrt(3) / 2 * r * (cos(theta) * sin(theta) * (1 - cos(phi)))) , 2)    
            + pow((-r/2 * cos(theta) * sin(phi) + L * cos(phi) + sqrt(3) / 2 * r * sin(phi) * sin(theta)), 2) 
            + pow(sqrt(3) / 2 * r - r / 2 * (cos(theta) * sin(theta) * (1 - cos(phi))) - sqrt(3) / 2 * r * (pow(cos(theta), 2) + cos(phi) * pow(sin(theta), 2)) + L * sin(phi) * sin(theta), 2)) - L) / .01;

  std::cout << "[x,y]: [" << x << "," << y <<"]"<<"\n";
  std::cout << "theta " << theta << "\n";
  std::cout << "A:" << A << "; B:" << B << "; C: " << C << '\n';
  // std::cout << "The goal angle is " << theta << "\n";
  move_position({{"outer_snake_cable_A", A}, 
                 {"outer_snake_cable_B", B}, 
                 {"outer_snake_cable_C", C}});
  print_goal();

}


void MedicalSnake::steer_outer(float x, float y) {
  set_mode(modes::STEERING);
  std::vector<std::string> motor_names = {"outer_snake_cable_A","outer_snake_cable_B","outer_snake_cable_C"};
  set_opmode(EXTENDED_POSITION_CONTROL_MODE, motor_names);
  std::vector<int32_t> profile_vel(motor_names.size(), max_cable_velocity_);
  set_profile_velocity(motor_names, profile_vel);
  
  //   set goals
  for (const std::string &name : motor_names)
  {
    if (!present_steer_center_outer_.count(name)) {
      present_steer_center_outer_.insert({name, get_fbk(name, "Present_Position")});
    }
    else{
      present_steer_center_outer_[name] = get_fbk(name, "Present_Position");
    }
  }

  // update_steer_angle_goal(x, y);

}

void MedicalSnake::update_steer_angle_goal(float x, float y) {
  // float x = steer_outer_x_;
  // float y = steer_outer_y_;

  x = -x;  
  double CR = .0047;
  // Finding phi in radians
  float phi = sqrt(pow(x, 2) + pow(y, 2)) * .25 * M_PI;

  if (phi > (M_PI / 6)) {
    phi = M_PI / 6;
  } 

  // Finding theta in radians
  float theta = atan2(y, x) - M_PI*.5;
  float L = .0085;
  float r = .0047;
  float co = 180.0;

  float A = co * (sqrt(2*pow(r, 2)*pow(cos(theta), 2)*(1-cos(phi)) + 2 * L * r * cos(theta) * sin(phi) + pow(L, 2)) - L) ;

  float B = co * (sqrt(pow((r/2 - r/2 * (cos(phi) * pow(cos(theta), 2) + pow(sin(theta), 2)) - L * cos(theta) * sin(phi) + sqrt(3) / 2 * r * (cos(theta) * sin(theta) * (1 - cos(phi)))) , 2)    
            + pow((r/2 * cos(theta) * sin(phi) - L * cos(phi) + sqrt(3) / 2 * r * sin(phi) * sin(theta)), 2) 
            + pow(sqrt(3) / 2 * r + r / 2 * (cos(theta) * sin(theta) * (1 - cos(phi))) - sqrt(3) / 2 * r * (pow(cos(theta), 2) + cos(phi) * pow(sin(theta), 2)) - L * sin(phi) * sin(theta) , 2)) - L) ;

  float C = co * (sqrt(pow((-r/2 + r/2 * (cos(phi) * pow(cos(theta), 2) + pow(sin(theta), 2)) + L * cos(theta) * sin(phi) + sqrt(3) / 2 * r * (cos(theta) * sin(theta) * (1 - cos(phi)))) , 2)    
            + pow((-r/2 * cos(theta) * sin(phi) + L * cos(phi) + sqrt(3) / 2 * r * sin(phi) * sin(theta)), 2) 
            + pow(sqrt(3) / 2 * r - r / 2 * (cos(theta) * sin(theta) * (1 - cos(phi))) - sqrt(3) / 2 * r * (pow(cos(theta), 2) + cos(phi) * pow(sin(theta), 2)) + L * sin(phi) * sin(theta), 2)) - L) ;

  std::cout << "[x,y]: [" << x << "," << y <<"]"<<"\n";
  std::cout << "theta " << theta << "\n";
  std::cout << "A:" << A << "; B:" << B << "; C: " << C << '\n';
  std::cout << "The goal angle is " << theta << "\n";

  int32_t goal_A = radian_to_value("outer_snake_cable_A",A) + present_steer_center_outer_["outer_snake_cable_A"];
  int32_t goal_B = radian_to_value("outer_snake_cable_B",B) + present_steer_center_outer_["outer_snake_cable_B"];
  int32_t goal_C = radian_to_value("outer_snake_cable_C",C) + present_steer_center_outer_["outer_snake_cable_C"];

  goals_ = {{"outer_snake_cable_A", goal_A}, 
            {"outer_snake_cable_B", goal_B}, 
            {"outer_snake_cable_C", goal_C}};
  // move_position_adjust_velocity({{"outer_snake_cable_A", goal_A}, 
  //                                {"outer_snake_cable_B", goal_B}, 
  //                                {"outer_snake_cable_C", goal_C}}); 

}

// ===============================================================================================



//================ Steer the outer snake in a stepwiese =======================

void MedicalSnake::steer_left()
{ // TODO(Maggie): Change the value by experiment if necessary
  set_mode(modes::STEERING);
  move_position({{"outer_snake_cable_B", 0.3489},
                 {"outer_snake_cable_C", -0.3489}});
  print_goal();
}


void MedicalSnake::steer_right()
{
  // TODO(Maggie): Change the value by experiment if necessary
  set_mode(modes::STEERING);
  move_position({{"outer_snake_cable_B", -0.3489}, 
                 {"outer_snake_cable_C", 0.3489}}); // value: ~33702
  print_goal();
}


void MedicalSnake::steer_up()
{ 
  // TODO(Maggie): Change the value by experiment if necessary
  set_mode(modes::STEERING);
  move_position({{"outer_snake_cable_B", -0.1744}, 
                 {"outer_snake_cable_C", -0.1744}, 
                 {"outer_snake_cable_A", 0.3489}});
}


void MedicalSnake::steer_down()
{ // TODO(Maggie): Change the value by experiment if necessary
  set_mode(modes::STEERING);
  move_position({{"outer_snake_cable_B", 0.1744}, 
                 {"outer_snake_cable_C", 0.1744}, 
                 {"outer_snake_cable_A", -0.3489}});
}

// ==========================================================================



// ============================== Home rail ====================================

void MedicalSnake::home_rail()
{
  // set_mode(modes::MOVING_POSITION);
  // float radians = 0.4;
  // move_position(outer_snake_cable, radians);
  // move_position({"inner_snake_cable"}, radians);
  set_mode(modes::HOMING_RAIL);
  set_opmode(VELOCITY_CONTROL_MODE, {"inner_snake_rail", "outer_snake_rail"});
  // current goal (force goal)
  goals_ = {{"inner_snake_rail", 1},
            {"outer_snake_rail", 1}};
}

// ============================================================================



void MedicalSnake::print_goal()
{ // a debug function
  for (auto goal : goals_)
  {
    std::cout << "goal is: " << goal.first <<": " << goal.second << std::endl;
  }
}


std::map<std::string, double> MedicalSnake::get_tension_fbk()
{
  std::map<std::string, double> result;
  for(const char *cable_name : {"outer_snake_cable_A", "outer_snake_cable_B", 
                               "outer_snake_cable_C", "inner_snake_cable"})
  {
    // result[cable_name] = twos_comp(get_fbk(cable_name,"Present_Current"), 16);
    // result[cable_name] = smooth_current_[cable_name];
    result[cable_name] = tension_reading(cable_name);
  }
  return result;
}

std::string MedicalSnake::get_snake_mode()
{
  switch (medsnake_mode_)
  {
  case modes::DEMO:
    return "Running Demo...";
    break;
  case modes::LOOSENING_CABLE:
    return "Loosening Cable...";
    break;
  case modes::TIGHTENING_CABLE:
    return "Tightening Cable by a step...";
    break;
  case modes::STEERING:
    return "Steering...";
    break;
  case modes::MOVING_FORWARD:
    return "Moving forward...";
    break;
  case modes::MOVING_BACKWARD:
    return "Moving backward...";
    break;
  case modes::MOVING_INNER:
    return "Moving inner snake...";
    break;
  case modes::MOVING_OUTER:
    return "Moving outer snake...";
    break;
  case modes::TIGHTENING_OUTER:
    return "Tightening outer snake to tension ...";
    break;
  case modes::LOOSENING_OUTER:
    return "Loosening outer snake...";
    break;
  case modes::TIGHTENING_INNER:
    return "Tightening inner snake to tension...";
    break;
  case modes::LOOSENING_INNER:
    return "Loosening inner snake...";
    break;
  case modes::HOMING_RAIL:  
    return "Homing Rail...";
    break;
  case modes::TIGHTENING:
    return "Tightening...";
    break;
  case modes::MOVING_POSITION:
    return "Moving in Position Control Mode";
    break;
  case modes::READY:
    return "Snake is Ready";
    break;
  default:
    return "";
  }
}