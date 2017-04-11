#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define RAD2DEG 3.14159265358979/180

double roll, pitch, yaw;
tf2::Quaternion q_tmp;

void imuCallback(const sensor_msgs::Imu& input){
	tf2::fromMsg(input.orientation, q_tmp);
    tf2::Matrix3x3(q_tmp).getRPY(roll, pitch, yaw);
    roll *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw *= RAD2DEG;
    printf("R: %.2f P: %.2f Y: %.2f wx: %.2f wy: %.2f wz: %.2f\n", roll, pitch, yaw, input.angular_velocity.x, input.angular_velocity.y, input.angular_velocity.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "print_euler");
	ros::NodeHandle nh;
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);


	ros::Rate loop_rate(2);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}