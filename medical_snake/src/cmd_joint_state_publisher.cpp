#include "cmd_joint_state_publisher.h"


CommandJointStatePublisher::CommandJointStatePublisher(size_t size) {
  init_subscriber();
  init_command_joint_state_publisher();
  position_.resize(size, 0.0); // 92 float num with value 0
}


void CommandJointStatePublisher::init_subscriber() // init a subscirber to user issued command
{
  sub_ = nh_.subscribe("joint_states", 100, 
                       &CommandJointStatePublisher::send_msg, this);
}

void CommandJointStatePublisher::init_command_joint_state_publisher()
{
  pub_ = nh_.advertise<sensor_msgs::JointState>("command_joint_state", 1);  
}

void CommandJointStatePublisher::send_msg(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState joint_msg;
  joint_msg.header = msg->header;
  joint_msg.name = name_;
  joint_msg.position.resize(92);
  
  for(int i = 0; i < position_.size(); i++)
  {
    joint_msg.position[i] = position_[i];
  }
  pub_.publish(joint_msg);
}
void CommandJointStatePublisher::steer_up()
{
  position_[2] += -0.3;
}
void CommandJointStatePublisher::steer_down()
{
  position_[2] += 0.3;
}
void CommandJointStatePublisher::steer_left()
{
  position_[3] += -0.3;
}
void CommandJointStatePublisher::steer_right()
{
  position_[3] += 0.3;
}
void CommandJointStatePublisher::forward_inner()
{
  position_[0] += 0.00923;
}
void CommandJointStatePublisher::forward_outer()
{
  position_[1] += 0.00923;
  follow_lead();

// void CommandJointStatePublisher::forward_both()
// {
//   position[1] += .00923;
//   position[0] += .00923;
// }

// void CommandJointStatePublisher::backward_both()
// {
//   position[0] -= .00923;
//   position[1] -= .00923;
// }


}
void CommandJointStatePublisher::backward_inner()
{
  position_[0] += -0.00923;
}
void CommandJointStatePublisher::backward_outer()
{
  position_[1] += -0.00923;
  unfollow_lead();
  
}
void CommandJointStatePublisher::follow_lead()
{ 
  for(int i = 88; i > 1; i--) { // propogation start backward from 88 to 2
    position_[i+3] = position_[i];
  }
  position_[2] = 0.0;
  position_[3] = 0.0;
  position_[4] = 0.0;
}
void CommandJointStatePublisher::unfollow_lead()
{ 
  for(int i = 2; i < 89; i++) { // propogation start from 2 to 88
    position_[i] = position_[i+3];
  }
  position_[89] = 0.0;
  position_[90] = 0.0;
  position_[91] = 0.0;
}
