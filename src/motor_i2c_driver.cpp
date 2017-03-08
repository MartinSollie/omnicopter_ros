#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <omnicopter_ros/MotorCommand.h>
#include <omnicopter_ros/PCA9685.h>
#include <omnicopter_ros/i2c.h>

void commandCallback(const omnicopter_ros::MotorCommand& input) {
	//Write command to PWM driver board
}


int main(int argc, char **argv){
	ros::init(argc, argv, "motor_i2c_driver");
	ros::NodeHandle nh;

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback);

	// Initialize I2C
	printf("Init i2c..\n");
	//READ TESTING:
<<<<<<< HEAD
	I2C *i2c = new I2C(1,70);
=======
	
	I2C *i2c = new I2C(1,7);
>>>>>>> e24173ad8b8f24dbd0b2c8a8c7a4af9cce37a15a

  while(1){
    int buf = 0;
	printf("Reading from i2c...\n");
    buf = i2c->read_byte(0x68);
    printf("IMU: %d\n", buf);
	}
	//END READ TESTING

	ros::spin();
}
