#include <ros/ros.h>
#include <omnicopter_ros/MotorCommand.h>

void commandCallback(const omnicopter_ros::MotorCommand& input) {
	//Write command to PWM driver board
}


int main(int argc, char **argv){
	ros::init(argc, argv, "motor_i2c_driver");
	ros::NodeHandle nh;

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback);

	// Initialize I2C

	ros::spin();
}