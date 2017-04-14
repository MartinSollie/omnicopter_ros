#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <Eigen/Geometry>

#define RAD2DEG 180/3.14159265358979

double roll, pitch, yaw;
Eigen::Quaterniond q_tmp;

void imuCallback(const sensor_msgs::Imu& input){

	q_tmp.x() = input.orientation.x;
	q_tmp.y() = input.orientation.y;
	q_tmp.z() = input.orientation.z;
	q_tmp.w() = input.orientation.w;

    Eigen::Vector3d euler = q_tmp.toRotationMatrix().eulerAngles(2, 1, 0);
  	yaw = euler[0]; pitch = euler[1]; roll = euler[2];
    roll *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw *= RAD2DEG;
    printf("R: %.2f P: %.2f Y: %.2f wx: % 04.3f wy: % 04.3f wz: % 04.3f\n", roll, pitch, yaw, input.angular_velocity.x, input.angular_velocity.y, input.angular_velocity.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "print_euler");
	ros::NodeHandle nh;
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);


	ros::Rate loop_rate(30);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}
