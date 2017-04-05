#include <ros/ros.h>
#include <omnicopter_ros/MotorCommand.h>
#include <omnicopter_ros/PCA9685.h>
#include <omnicopter_ros/I2C.h>

PCA9685 motorPWM(1,0x40);


void commandCallback(const omnicopter_ros::MotorCommand& input) {
	//Write command to PWM driver board
        motorPWM.setPWM(1, input.motor1_usec);
        motorPWM.setPWM(2, input.motor2_usec);
        motorPWM.setPWM(3, input.motor3_usec);
        motorPWM.setPWM(4, input.motor4_usec);
        motorPWM.setPWM(5, input.motor5_usec);
        motorPWM.setPWM(6, input.motor6_usec);
        motorPWM.setPWM(7, input.motor7_usec);
        motorPWM.setPWM(8, input.motor8_usec);

}


int main(int argc, char **argv){
	ros::init(argc, argv, "motor_i2c_driver");
	ros::NodeHandle nh;

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback);

	motorPWM.setPWM(1, 900);
	motorPWM.setPWM(2, 900);
	motorPWM.setPWM(3, 900);
	motorPWM.setPWM(4, 900);
	motorPWM.setPWM(5, 900);
	motorPWM.setPWM(6, 900);
	motorPWM.setPWM(7, 900);
	motorPWM.setPWM(8, 900);
	ros::spin();
}
