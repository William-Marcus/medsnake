/*
 * The main driver ros node to control the medsnake based on user issued
 * commands. Once its listener gets a command, the node processes the command
 * and calls proper functions to move the medical snake and stimulate the snake's
 * movement and shape in Rviz
 */

#include "medsnake_control.h"
#include "cmd_joint_state_publisher.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "medsnake_main_node");
  SnakeControl control(argv[1], argv[2], argv[3]);
  CommandJointStatePublisher joint_publisher(92);

  while(ros::ok())
  {
    const uint32_t num_loop = 100;
    auto start_time = std::chrono::high_resolution_clock::now();
    for(int i=0; i<num_loop; ++i)
    {
      control.snake_update(); // snake update: check goal and write to register
      control.publish_tension_reading();
      control.publish_snake_mode();
      // joint_publisher.send_msg();
      

      if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'o')
      {
        ROS_INFO("Stop!");
        control.emergency_stop();

      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'q')
      {
        control.demo();
      }
      // Advance
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'w')
      {
        control.advance();
      }

      // Retract
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 's')
      {
        control.retract();
      }

      // Steer Left
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'a'
              && control.snake_is_ready())
      {
        ROS_INFO("Steering Left ...");
        control.steer_left();
        joint_publisher.steer_left();
      }

      // Steer Right
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'd'
              && control.snake_is_ready())
      {
        ROS_INFO("Steer Right ...");
        control.steer_right();
        joint_publisher.steer_right();
      }

      // Steer Up
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'y'
              && control.snake_is_ready())
      {
        ROS_INFO("Steering Up ...");
        control.steer_up();
        joint_publisher.steer_up();
      }

      // Steer Down
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'h'
              && control.snake_is_ready())
      {
        ROS_INFO("Steer Down ...");
        control.steer_down();
        joint_publisher.steer_down();
      }

      // Tighten Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 't'
              && control.snake_is_ready())
      {
        ROS_INFO("Tighten Outer ...");
        control.tighten_outer();
      }

      // Loosen Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'g'
              && control.snake_is_ready())
      {
        ROS_INFO("Loosen Outer ...");
        control.loosen_outer();
      }

      // Tighten Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'v'
              && control.snake_is_ready())
      {
        ROS_INFO("Tighten Inner ...");
        control.tighten_inner();
      }

      // Loosen Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'b'
              && control.snake_is_ready())
      {
        ROS_INFO("Loosen Inner ...");
        control.loosen_inner();
      }

      // Forward Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'e'
              && control.snake_is_ready())
      {
        ROS_INFO("Forward Inner ...");
        control.forward_inner();
        joint_publisher.forward_inner();
      }

      // Backward Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'c'
              && control.snake_is_ready())
      {
        ROS_INFO("Backward Inner ...");
        control.backward_inner();
        joint_publisher.backward_inner();
      }

      // Forward Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'u'
              && control.snake_is_ready())
      {
        ROS_INFO("Forward Outer ...");
        control.forward_outer();
        joint_publisher.forward_outer();
      }

      // Backward Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'm'
              && control.snake_is_ready())
      {
        ROS_INFO("Backward Outer ...");
        control.backward_outer();
        joint_publisher.backward_outer();
      }

      // Home Rail
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'r'
              && control.snake_is_ready())
      {
        ROS_INFO("Homing Rail ...");
        control.home_rail();
      }

      // Tighten Outer Snake Cable A
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'i'
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable A ...");
        control.tighten_outer_A();
      }

      // Tighten Outer Snake Cable B
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'k'
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable B ...");
        control.tighten_outer_B();
      }

      // Tighten Outer Snake Cable C
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'j'
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable C ...");
        control.tighten_outer_C();
      }

      // Loosen Outer Snake Cable A
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'p'
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable A ...");
        control.loosen_outer_A();
      }

      // Loosen Outer Snake Cable B
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'l'
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable B ...");
        control.loosen_outer_B();
      }

      // Loosen Outer Snake Cable C
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'n'
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable C ...");
        control.loosen_outer_C();
      }




      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'z'
              && control.snake_is_ready())
      {
        ROS_INFO("Steering to angle...");
        control.steer_angle();
      }
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == 'z'
              && control.snake_is_steering())
      {
        ROS_INFO("Steering to angle...");
        control.update_steer_angle();
      }
      else if(control.cmd_queue_empty() && control.snake_is_steering())
      {
        ROS_INFO("Switching from steering back to ready ...");
        control.emergency_stop();
      }

      // if (control.cmd_queue_empty()){
      //   std::cout << " control.get_cmd_queue_top() " <<  "EMPTY!" << "\n"; 
      // }
      // else{
      //   std::cout << " control.get_cmd_queue_top() " <<  control.get_cmd_queue_top() << "\n";
      // }

      ros::spinOnce();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto dt_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    double avg_time = (dt_seconds.count()/1000.0)/(float)num_loop;

    std::cout << "Average time to run update(): " << avg_time << std::endl;
    std::cout << "Update rate: " << 1 / avg_time << std::endl;

    
  }
}
