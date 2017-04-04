#include <ros/ros.h>
#include <omnicopter_ros/MotorCommand.h>
#include <iostream>

int main(int argc, char **argv){
	ros::init(argc, argv, "motor_tester");
	ros::NodeHandle nh;

	ros::Publisher motor_pub = nh.advertise<omnicopter_ros::MotorCommand>("motor_commands",0);

	std::cout << "Input syntax:" << std::endl;
	std::cout << "- Disarm all motors: 'stop'" << std::endl;
	std::cout << "- Set speed for all motors: 'all <usec>'" << std::endl;
	std::cout << "- Set speed for one motor: 'set <motor> <usec>'" << std::endl;

	std::string input;
	std::string command, tmp;
	omnicopter_ros::MotorCommand msg;

	while(1)
	{
		std::queue<std::string> v;

		getline(std::cin, input);
		std::istringstream iss(input);
		getline( iss, command, ' ' );
		if(command == "stop"){
			while ( getline( iss, tmp, ' ' ) ) {
				v.push(tmp);
			}
			if(v.size() != 0){
				std::cout << "Syntax error: 0 arguments expected" << std::endl;
				continue;
			}
			msg.motor1_usec = 900;
			msg.motor2_usec = 900;
			msg.motor3_usec = 900;
			msg.motor4_usec = 900;
			msg.motor5_usec = 900;
			msg.motor6_usec = 900;
			msg.motor7_usec = 900;
			msg.motor8_usec = 900;
			msg.header.stamp = ros::Time::now();
			motor_pub.publish(msg);
		}
		else if(command == "all"){
			while ( getline( iss, tmp, ' ' ) ) {
				v.push(tmp);
			}
			if(v.size() != 1){
				std::cout << "Syntax error: 1 argument expected" << std::endl;
				continue;
			}
			msg.motor1_usec = std::atof(v.front().c_str());
			msg.motor2_usec = std::atof(v.front().c_str());
			msg.motor3_usec = std::atof(v.front().c_str());
			msg.motor4_usec = std::atof(v.front().c_str());
			msg.motor5_usec = std::atof(v.front().c_str());
			msg.motor6_usec = std::atof(v.front().c_str());
			msg.motor7_usec = std::atof(v.front().c_str());
			msg.motor8_usec = std::atof(v.front().c_str());
			msg.header.stamp = ros::Time::now();
			motor_pub.publish(msg);
		}
		else if(command == "set"){
			while ( getline( iss, tmp, ' ' ) ) {
				v.push(tmp);
			}
			if(v.size() != 2){
				std::cout << "Syntax error: 2 arguments expected" << std::endl;
				continue;
			}
			int motor_index = std::atof(v.front().c_str());
			v.pop();
			if(motor_index < 1 || motor_index > 8){
				std::cout << "Invalid motor index" << std::endl;
				continue;
			}
			int motor_usec = std::atof(v.front().c_str());
			if(motor_usec < 800 || motor_usec > 2200){
				std::cout << "Invalid motor command" << std::endl;
				continue;
			} 

			switch(motor_index){
				case 1:
					msg.motor1_usec = motor_usec;
					break;
				case 2:
					msg.motor1_usec = motor_usec;
					break;
				case 3:
					msg.motor1_usec = motor_usec;
					break;
				case 4:
					msg.motor1_usec = motor_usec;
					break;
				case 5:
					msg.motor1_usec = motor_usec;
					break;
				case 6:
					msg.motor1_usec = motor_usec;
					break;
				case 7:
					msg.motor1_usec = motor_usec;
					break;
				case 8:
					msg.motor1_usec = motor_usec;
					break;
			}
			msg.header.stamp = ros::Time::now();
			motor_pub.publish(msg);
		}
		else{ 
			std::cout << "Syntax error: command not recognized" << std::endl;
		}
	}


			