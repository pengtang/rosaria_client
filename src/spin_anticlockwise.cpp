#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Spin_Anti_Clockwisely");
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
  geometry_msgs::Twist msg;

  double BASE_SPEED = 0.2, MOVE_TIME = 3.0, CLOCK_SPEED = 0.5, PI = 3.14159;
  int count = 0;
  ros::Rate rate(CLOCK_SPEED);

  while(ros::ok() && count<MOVE_TIME/CLOCK_SPEED + 1)
    {
      // Spin PI/4
      msg.angular.z = 2 * PI/ int(MOVE_TIME/CLOCK_SPEED) / 4;
      pub.publish(msg);
      ROS_INFO_STREAM("The robot is now spinning anti-clockwisely!");
      count++;
      ros::spinOnce();
      rate.sleep();
    }    
    // Stop the spin
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    ROS_INFO_STREAM("The robot finished spinning 90 degree!");
}
