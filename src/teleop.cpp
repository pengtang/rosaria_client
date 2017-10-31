#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_A 0x61
#define KEYCODE_Z 0x7A
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class TeleopRosAria
{
  public:
    TeleopRosAria();
    void keyLoop();
  private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    double current_linear_, current_angular_, step_linear_, step_angular_;
    ros::Publisher twist_pub_;
};
TeleopRosAria::TeleopRosAria():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0),
  current_angular_(0.1),
  current_linear_(0.1),
  step_linear_(0.2),
  step_angular_(0.1)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_RosAria");
  TeleopRosAria teleop_RosAria;
  signal(SIGINT,quit);
  teleop_RosAria.keyLoop();
  return(0);
}
void TeleopRosAria::keyLoop()
{
  char c;
  bool dirty=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  puts("a/z - Increase/decrease linear velocity");
  puts("s/x - Increase/decrease angular velocity");
  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }
    linear_=angular_=0;
    char printable[100];
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
    	  angular_ = current_angular_;
    	  linear_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
    	  angular_ = -current_angular_;
    	  linear_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_U:
    	  ROS_DEBUG("UP");
    	  linear_ = current_linear_;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
    	  linear_ = -current_linear_;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
	case KEYCODE_A:
	  ROS_DEBUG("INCREASE LINEAR SPEED");
	  current_linear_ += step_linear_;
	  sprintf(printable, "Linear speed: %02f", current_linear_);
	  puts(printable);
	  dirty=true;
	  break;
	case KEYCODE_Z:
	  ROS_DEBUG("DECREASE LINEAR SPEED");
          current_linear_ -= step_linear_;
	  if(current_linear_ < 0)
	      current_linear_ = 0;
          sprintf(printable, "Linear speed: %02f", current_linear_);
          puts(printable);
          dirty=true;
          break;
	case KEYCODE_S:
	  ROS_DEBUG("INCREASE ANGULAR SPEED");
          current_angular_ += step_angular_;
          sprintf(printable, "Angular speed: %02f", current_angular_);
          puts(printable);
          dirty=true;
          break;
	case KEYCODE_X:
	  ROS_DEBUG("DECREASE LINEAR SPEED");
          current_angular_ -= step_angular_;
	  if(current_angular_ < 0)
	      current_angular_ = 0;
          sprintf(printable, "Angular speed: %02f", current_angular_);
          puts(printable);
          dirty=true;
          break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
    	  linear_ = 0;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        ROS_INFO_STREAM("You quit the teleop successfully");
        return;
        break;
  	}
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
  	{
  	  twist_pub_.publish(twist);
  	  dirty=false;
  	}
  }
  return;
}
