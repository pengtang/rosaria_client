#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#define KEYCODE_1 0x31 
#define KEYCODE_2 0X32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_Q 0X71

int main(int argc, char **argv)
{

	ros::init(argc, argv, "RosAria_interface");
	ros::NodeHandle nh;

	std::cout  	<< "******************************************************************" << std::endl
				<< "*                   ROSARIA CLIENT INTERFACE                     *" << std::endl
				<< "*                                                                *" << std::endl
				<< "*            Welcome to the RosAria client interface!            *" << std::endl
				<< "*                                                                *" << std::endl
				<< "*       [1] go_three_second                                      *" << std::endl
				<< "*       [2] spin_clockwise                                       *" << std::endl
				<< "*       [3] spin_counterclockwise                                *" << std::endl
				<< "*       [4] teleop                                               *" << std::endl
				<< "*       [5] print_state                                          *" << std::endl
				<< "*       [6] enable_motors                                        *" << std::endl
				<< "*       Press [Q] to close the interface                         *" << std::endl 
				<< "******************************************************************" << std::endl;

	char select,a;

	while(ros::ok()){
		std::cout << "Please select a program to run, or hit q to quit: ";
		std::cin >> select;	
		switch(select)
		{
			case KEYCODE_1:
				a = system("rosrun rosaria_client go_three_second");
				break;
			case KEYCODE_2:
				a = system("rosrun rosaria_client spin_clockwise");
				break;
			case KEYCODE_3:
				a = system("rosrun rosaria_client spin_counterclockwise");
				break;
			case KEYCODE_4:
				a = system("rosrun rosaria_client teleop");
				break;
			case KEYCODE_5:
				a = system("rosrun rosaria_client print_state");
				break;
			case KEYCODE_6:
				a = system("rosrun rosaria_client enable_motors");
				break;
			case KEYCODE_Q:
				return false;
				break;
			default:
			std::cout << "Please enter a number from the list or Q to quit." << std::endl;
		}
	}
	return 1;
}
