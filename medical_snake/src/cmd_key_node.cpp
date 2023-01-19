#include <ros/ros.h>
#include "std_msgs/Char.h"
 
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

 
// Reminder message
const char* reminder = R"(
 
Medsnake Control Command by Key Press (Last updated 01/13/2023)
---------------------------
a : steer left
b : loosen inner
c : back inner
d : steer right
e : forward inner
f : backward both
g : loosen outer
h : steer down
i : tighten A
j : tighten C
k : tighten B
l : loosen B
m : backward outer
n : loosen C
o : stop
p : loosen A
q : demo
r : homing
s : retract
t : tighter outer
u : forward outer
v : tighten inner
w : advance
x : **open for use**
y : steer up
z : steer angle
 
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