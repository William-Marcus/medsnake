/*
 * A class that subscribe to joint state message published by the joint state
 * publisher and construct a similar custom joint state message based on user 
 * issued command to simulate medical snake movement in rviz
 */

#include <vector>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


class CommandJointStatePublisher
{
 public:
  CommandJointStatePublisher(size_t size);
  
  /// Initialize a subscirber to joint state topic
  void init_subscriber(); 
  
  /// Initialize the custom joint state message publisher
  void init_command_joint_state_publisher();
  
  /// The callback function that publish the custom message
  /// upon receiving joint state message
  void send_msg(const sensor_msgs::JointState::ConstPtr& msg);
  
  /// Issue steer up joint state message
  void steer_up();

  /// Issue steer down joint state message
  void steer_down();
  
  /// Issue steer left joint state message
  void steer_left();

  /// Issue steer right joint state message
  void steer_right();

  /// Issue forward inner joint state message
  void forward_inner();
  
  /// Issue forward outer joint state message
  void forward_outer();
  
  /// Issue backward inner joint state message
  void backward_inner();
  
  /// Issue backward outer joint state message
  void backward_outer();
  
  /// Issue follow leader joint state message
  void follow_lead();
  
  /// Issue unfollow leader joint state message
  void unfollow_lead();

  // Issue both forward joint state message
  void backward_both();

  // Issue both backward joint state message
  void forward_both();

  // Issue both backward joint state message
  void forward_both_cont();

  


 private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;
  const std::vector<std::string> name_ =
  {
    "joint_inner",
    "joint_outer",
    "balljoint_30_0",
    "balljoint_30_1",
    "balljoint_30_2",
    "balljoint_29_0",
    "balljoint_29_1",
    "balljoint_29_2",
    "balljoint_28_0",
    "balljoint_28_1",
    "balljoint_28_2",
    "balljoint_27_0",
    "balljoint_27_1",
    "balljoint_27_2",
    "balljoint_26_0",
    "balljoint_26_1",
    "balljoint_26_2",
    "balljoint_25_0",
    "balljoint_25_1",
    "balljoint_25_2",
    "balljoint_24_0",
    "balljoint_24_1",
    "balljoint_24_2",
    "balljoint_23_0",
    "balljoint_23_1",
    "balljoint_23_2",
    "balljoint_22_0",
    "balljoint_22_1",
    "balljoint_22_2",
    "balljoint_21_0",
    "balljoint_21_1",
    "balljoint_21_2",
    "balljoint_20_0",
    "balljoint_20_1",
    "balljoint_20_2",
    "balljoint_19_0",
    "balljoint_19_1",
    "balljoint_19_2",
    "balljoint_18_0",
    "balljoint_18_1",
    "balljoint_18_2",
    "balljoint_17_0",
    "balljoint_17_1",
    "balljoint_17_2",
    "balljoint_16_0",
    "balljoint_16_1",
    "balljoint_16_2",
    "balljoint_15_0",
    "balljoint_15_1",
    "balljoint_15_2",
    "balljoint_14_0",
    "balljoint_14_1",
    "balljoint_14_2",
    "balljoint_13_0",
    "balljoint_13_1",
    "balljoint_13_2",
    "balljoint_12_0",
    "balljoint_12_1",
    "balljoint_12_2",
    "balljoint_11_0",
    "balljoint_11_1",
    "balljoint_11_2",
    "balljoint_10_0",
    "balljoint_10_1",
    "balljoint_10_2",
    "balljoint_9_0",
    "balljoint_9_1",
    "balljoint_9_2",
    "balljoint_8_0",
    "balljoint_8_1",
    "balljoint_8_2",
    "balljoint_7_0",
    "balljoint_7_1",
    "balljoint_7_2",
    "balljoint_6_0",
    "balljoint_6_1",
    "balljoint_6_2",
    "balljoint_5_0",
    "balljoint_5_1",
    "balljoint_5_2",
    "balljoint_4_0",
    "balljoint_4_1",
    "balljoint_4_2",
    "balljoint_3_0",
    "balljoint_3_1",
    "balljoint_3_2",
    "balljoint_2_0",
    "balljoint_2_1",
    "balljoint_2_2",
    "balljoint_1_0",
    "balljoint_1_1",
    "balljoint_1_2"
  };
  std::vector<double> position_; 
};
