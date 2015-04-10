#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Interface");
	ros::NodeHandle nh;

	ROS_INFO_STREAM("Please enter 1 or 2");

	int i;
	//int cond;
	//cin >> cond;

	//if(cond == 1)
		i = system("rosrun rosaria_client go_three_second");
	//else if (cond == 2)
	//	i = system("rosrun rosaria_client spin_clockwise");

	return 1;
}