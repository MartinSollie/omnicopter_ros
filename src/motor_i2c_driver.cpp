#include <ros/ros.h>
#include <omnicopter_ros/MotorCommand.h>
#include <omnicopter_ros/PCA9685.h>
#include <omnicopter_ros/I2C.h>
#include <omnicopter_ros/RCInput.h>

bool armed;
PCA9685 motorPWM(1,0x40);

#define MIN_THROTTLE 30
#define MAX_THROTTLE 150

int limitPWM(int input){
	if(input < 1500+MIN_THROTTLE && input > 1500-MIN_THROTTLE){
		return 1500;
	}
	if(input > 1500+MAX_THROTTLE){
		return 1500+MAX_THROTTLE;
	}
	if(input < 1500-MAX_THROTTLE){
		return 1500-MAX_THROTTLE;
	}
	return input;
}

void rcCallback(const omnicopter_ros::RCInput& input){
	if(armed && !input.arm){
		motorPWM.setPWM(1, 1500);
    	motorPWM.setPWM(2, 1500);
    	motorPWM.setPWM(3, 1500);
    	motorPWM.setPWM(4, 1500);
    	motorPWM.setPWM(5, 1500);
    	motorPWM.setPWM(6, 1500);
    	motorPWM.setPWM(7, 1500);
    	motorPWM.setPWM(8, 1500);
	ROS_WARN("Motors disarmed");
	}
	if(!armed && input.arm){
		ROS_WARN("Motors armed");
	}
	armed = input.arm;
}

void commandCallback(const omnicopter_ros::MotorCommand& input) {
	//Write command to PWM driver board
	if(armed){
		motorPWM.setPWM(1, limitPWM(input.motor1_usec));
    		motorPWM.setPWM(2, limitPWM(input.motor2_usec));
    		motorPWM.setPWM(3, limitPWM(input.motor3_usec));
    		motorPWM.setPWM(4, limitPWM(input.motor4_usec));
    		motorPWM.setPWM(5, limitPWM(input.motor5_usec));
    		motorPWM.setPWM(6, limitPWM(input.motor6_usec));
    		motorPWM.setPWM(7, limitPWM(input.motor7_usec));
    		motorPWM.setPWM(8, limitPWM(input.motor8_usec));
	}
	else{
	motorPWM.setPWM(1, 1500);
        motorPWM.setPWM(2, 1500);
        motorPWM.setPWM(3, 1500);
        motorPWM.setPWM(4, 1500);
        motorPWM.setPWM(5, 1500);
        motorPWM.setPWM(6, 1500);
        motorPWM.setPWM(7, 1500);
        motorPWM.setPWM(8, 1500);
	}
    

}


int main(int argc, char **argv){
	ros::init(argc, argv, "motor_i2c_driver");
	ros::NodeHandle nh;

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback);
	ros::Subscriber rc_sub = nh.subscribe("rc_input",1,rcCallback);

	motorPWM.setPWM(1, 1500);
	motorPWM.setPWM(2, 1500);
	motorPWM.setPWM(3, 1500);
	motorPWM.setPWM(4, 1500);
	motorPWM.setPWM(5, 1500);
	motorPWM.setPWM(6, 1500);
	motorPWM.setPWM(7, 1500);
	motorPWM.setPWM(8, 1500);
	ros::spin();
}
