#include <ros/ros.h>
#include <omnicopter_ros/RCInput.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "motor_i2c_driver");
	ros::NodeHandle nh("~");

	ros::Publisher pub_rc = nh.advertise<omnicopter_ros::RCInput>("rc_input", 5); 

	// Initialize I2C RC stuff


	ros::Rate loop_rate(50);
	omnicopter_ros::RCInput msg;

	while(ros::ok()){

		// Read data from RC

		msg.header.stamp = ros::Time::now();
		// other message content

		pub_rc.publish(msg);

		loop_rate.sleep();
	}

}