#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <rosaria/BumperState.h>
#include <iomanip> // for std :: setprecision and std :: fixed

// A callback function. Executed each time a new pose // message arrives .
void poseMessageReceived(const nav_msgs::Odometry& msg) 
{
	std::cout<< std::setprecision(2) << std::fixed <<
		"Current position=(" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y << ") " << 
		"Current direction=" << msg.pose.pose.orientation.x << msg.pose.pose.orientation.y << msg.pose.pose.orientation.w<<"\r";
	std::flush(std::cout);
}

//void bumperStateMessageReceived(const rosaria::BumperState &msg)
//{
//	ROS_INFO_STREAM("The front bumpers are "<<msg.front_bumpers<<endl<<"The rear bumpers are "<<msg.rear_bumpers);
//}

void batteryStateOfChargeMessageReceived(const std_msgs::Float64 msg)
{
	ROS_INFO_STREAM("The battery state of charge is "<<msg);
}

void batteryVoltageMessageReceived(const std_msgs::Float32 msg)
{
	ROS_INFO_STREAM("The battery voltage is "<< msg);
}

void batteryChargeStateMessageReceived(const std_msgs::Int8 msg)
{
	ROS_INFO_STREAM("The battery charge state message received is "<< msg);
}
void motorsStateMessageReceived(const std_msgs::Bool msg)
{
	if (msg.data)
		ROS_INFO_STREAM("The motor is good");
	else
		ROS_INFO_STREAM("The motor is bad");
}
int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "print_aria_state"); ros::NodeHandle nh;
	// Create a subscriber object .
	ros::Subscriber pose, battery_state_of_charge, battery_voltage, battery_charge_state, motors_state;
	pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived) ;
	//bumper_state = nh.subscribe("RosAria/bumper_state", 1000, &bumperStateMessageReceived) ;
	battery_state_of_charge = nh.subscribe("RosAria/bumper_state_of_charge", 1000, &batteryStateOfChargeMessageReceived) ;
	battery_voltage = nh.subscribe("RosAria/battery_voltage", 1000, &batteryVoltageMessageReceived) ;
	battery_charge_state = nh.subscribe("RosAria/battery_charge_state", 1000, &batteryChargeStateMessageReceived) ;
	motors_state = nh.subscribe("RosAria/motors_state", 1000, &motorsStateMessageReceived) ;
	// Let ROS take over.
	ros::spin(); 
}
