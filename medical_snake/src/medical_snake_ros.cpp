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
      control.publish_motor_position();
      control.publish_snake_mode();
      // joint_publisher.send_msg();
      

      if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "stop")
      {
        ROS_INFO("Stop!");
        control.emergency_stop();

      }

      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "inner_tension_cont" && control.snake_is_ready())
      {
        control.tighten_inner_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_a_tension_cont" && control.snake_is_ready())
      {
        control.tighten_outer_A_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_b_tension_cont" && control.snake_is_ready())
      {
        control.tighten_outer_B_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_c_tension_cont" && control.snake_is_ready())
      {
        control.tighten_outer_C_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "inner_loosen_cont" && control.snake_is_ready())
      {
        control.loosen_inner_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_a_loosen_cont" && control.snake_is_ready())
      {
        control.loosen_outer_A_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_b_loosen_cont" && control.snake_is_ready())
      {
        control.loosen_outer_B_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_c_loosen_cont" && control.snake_is_ready())
      {
        control.loosen_outer_C_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_tension_cont" && control.snake_is_ready())
      {
        control.tighten_outer_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "outer_loosen_cont" && control.snake_is_ready())
      {
        control.loosen_outer_cont();
      }
      //------------------------------------------------------------------------------------------
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_both" && control.snake_is_ready())
      {
        control.forward_both();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_both_cont" && control.snake_is_ready())
      {
        control.forward_both_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_both_cont" && control.snake_is_ready())
      {
        control.backward_both_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_both" && control.snake_is_ready())
      {
        control.backward_both();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_outer_cont" && control.snake_is_ready())
      {
        control.backward_outer_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_outer_cont" && control.snake_is_ready())
      {
        control.forward_outer_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_inner_cont" && control.snake_is_ready())
      {
        control.backward_inner_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_inner_cont" && control.snake_is_ready())
      {
        control.forward_inner_cont();
      }
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "demo" && control.snake_is_ready())
      {
        control.demo();
      }
      // Advance
      else if (!control.cmd_queue_empty() && control.get_cmd_queue_top() == "advance" && control.snake_is_ready())
      {
        control.advance();
      }

      // Retract
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "retract" && control.snake_is_ready())
      {
        control.retract();
      }

      // Steer Left
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer_left"
              && control.snake_is_ready())
      {
        ROS_INFO("Steering Left ...");
        control.steer_left();
        joint_publisher.steer_left();
      }

      // Steer Right
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer_right"
              && control.snake_is_ready())
      {
        ROS_INFO("Steer Right ...");
        control.steer_right();
        joint_publisher.steer_right();
      }

      // Steer Up
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer_up"
              && control.snake_is_ready())
      {
        ROS_INFO("Steering Up ...");
        control.steer_up();
        joint_publisher.steer_up();
      }

      // Steer Down
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer_down"
              && control.snake_is_ready())
      {
        ROS_INFO("Steer Down ...");
        control.steer_down();
        joint_publisher.steer_down();
      }

      // Tighten Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "tight_outer"
              && control.snake_is_ready())
      {
        ROS_INFO("Tighten Outer ...");
        control.tighten_outer();
      }

      // Loosen Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "loose_outer"
              && control.snake_is_ready())
      {
        ROS_INFO("Loosen Outer ...");
        control.loosen_outer();
      }

      // Tighten Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "tight_inner"
              && control.snake_is_ready())
      {
        ROS_INFO("Tighten Inner ...");
        control.tighten_inner();
      }

      // Loosen Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "loose_inner"
              && control.snake_is_ready())
      {
        ROS_INFO("Loosen Inner ...");
        control.loosen_inner();
      }

      // Forward Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_inner"
              && control.snake_is_ready())
      {
        ROS_INFO("Forward Inner ...");
        control.forward_inner();
        joint_publisher.forward_inner();
      }

      // Backward Inner
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_inner"
              && control.snake_is_ready())
      {
        ROS_INFO("Backward Inner ...");
        control.backward_inner();
        joint_publisher.backward_inner();
      }

      // Forward Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "fwd_outer"
              && control.snake_is_ready())
      {
        ROS_INFO("Forward Outer ...");
        control.forward_outer();
        joint_publisher.forward_outer();
      }

      // Backward Outer
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "back_outer"
              && control.snake_is_ready())
      {
        ROS_INFO("Backward Outer ...");
        control.backward_outer();
        joint_publisher.backward_outer();
      }

      // Home Rail
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "home_rail"
              && control.snake_is_ready())
      {
        ROS_INFO("Homing Rail ...");
        control.home_rail();
      }

      // Home
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "homing"
              && control.snake_is_ready())
      {
        ROS_INFO("Homing the Entire Snake ...");
        control.home();
      }

      // Tighten Outer Snake Cable A
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "tight_outer_A"
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable A ...");
        control.tighten_outer_A();
      }

      // Tighten Outer Snake Cable B
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "tight_outer_B"
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable B ...");
        control.tighten_outer_B();
      }

      // Tighten Outer Snake Cable C
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "tight_outer_C"
              && control.snake_is_ready())
      {
        ROS_INFO("Tightening Outer Snake Cable C ...");
        control.tighten_outer_C();
      }

      // Loosen Outer Snake Cable A
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "loose_outer_A"
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable A ...");
        control.loosen_outer_A();
      }

      // Loosen Outer Snake Cable B
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "loose_outer_B"
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable B ...");
        control.loosen_outer_B();
      }

      // Loosen Outer Snake Cable C
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "loose_outer_C"
              && control.snake_is_ready())
      {
        ROS_INFO("Loosening Outer Snake Cable C ...");
        control.loosen_outer_C();
      }




      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer"
              && control.snake_is_ready())
      {
        ROS_INFO("Steering to angle...");
        control.steer_angle();
      }
      else if(!control.cmd_queue_empty() && control.get_cmd_queue_top() == "steer"
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
