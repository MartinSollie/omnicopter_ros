#include <ros/ros.h>
#include <omnicopter_ros/I2C.h>
#include <std_msgs/Float32.h>

#define I2C_SLAVE 0x55

int main(int argc, char **argv){
	ros::init(argc, argv, "sonar_driver");
 	ros::NodeHandle nh("~");

	ros::Publisher pub_sonar = nh.advertise<std_msgs::Float32>("sonar_range", 1);

	//Initialize I2C RC stuff
	I2C rc_i2c(1, I2C_SLAVE);

	uint8_t data[3];

	ros::Rate loop_rate(20);
	std_msgs::Float32 msg;

	while(ros::ok()){
		if(rc_i2c.read_block(0, data, 3)){
			uint8_t byte_h = data[0];
			uint8_t byte_l = data[1];
			uint8_t range = byte_l | byte_h << 8;

			msg.data = 0.01*range; //cm to m
			pub_sonar.publish(msg);


		}
		loop_rate.sleep();
	}
}
