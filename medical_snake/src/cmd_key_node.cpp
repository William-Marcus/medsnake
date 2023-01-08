#include <ros/ros.h>
#include "std_msgs/Char.h"
 
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

 
// Reminder message
const char* reminder = R"(
 
Medsnake Control Command by Key Press
---------------------------
w: Advance
s: Retract
a: Steer Left
d: Steer Right
y: Steer Up
h: Steer Down
t: Tighten Outer
g: Loosen Outer
v: Tighten Inner
b: Loosen Inner
q: Demo
o: Emergency Stop
z: Joystick Steering
 
CTRL-C to quit
 
)";
 
 // Init variables

char key(' ');
 
 // For non-blocking keyboard inputs
int getch(void)
 {
    int ch;
    struct termios oldt;
    struct termios newt;
 
    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
 
    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);
  
    // Get the current character
    ch = getchar();
 
 // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
 
    return ch;
 }
 
 int main(int argc, char** argv) {
    // Init ROS node
    ros::init(argc, argv, "teleop_keyboard");
    ros::NodeHandle nh;
 
    // Init key_pressed publisher
    ros::Publisher pub = nh.advertise<std_msgs::Char>("key_pressed", 1);
 
    // Create message
    std_msgs::Char msg;
 
    printf("%s", reminder);
 
    while(true){
 
        // Get the pressed key
        key = getch();


        // If ctrl-C (^C) was pressed, terminate the program
        if (key == '\x03') {
            printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
            break;
        }
        msg.data = key;
        pub.publish(msg);
        ros::spinOnce();
   }
 
   return 0;
 }