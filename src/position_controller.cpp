#include <ros/ros.h>
#include <omnicopter_ros/RCInput.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

ros::Publisher force_pub;
Eigen::Matrix3d R; // v_body = R.inverse()*v_enu

void setpointCallback(const omnicopter_ros::RCInput& input){
	geometry_msgs::Vector3Stamped msg;
	if(input.rc_mode.position_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_BODY_UP){
		msg.vector.x = 0;
		msg.vector.y = 0;
		msg.vector.z = (input.throttlestick+1)*5; // 0-10 Newtons
	}
	else if(input.rc_mode.position_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_BODY){
		msg.vector.x = input.pitchstick*3;// -3 - 3 N
		msg.vector.y = -input.rollstick*3; // -3 - 3 N
		msg.vector.z = (input.throttlestick+1)*5; // 0-10 N
	}
	else if(input.rc_mode.position_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_ENU){
		Eigen::Vector3d force_body = R.inverse()*Eigen::Vector3d(input.pitchstick*3, -input.rollstick*3, 5*(input.throttlestick+1));
		msg.vector.x = force_body(0);
		msg.vector.y = force_body(1);
		msg.vector.z = force_body(2);
	}
	else {
		ROS_ERROR("Position controller: unknown setpoint type");
		return;
	}
	msg.header.stamp = ros::Time::now();
	force_pub.publish(msg);
}

void imuCallback(const sensor_msgs::Imu& input){
	Eigen::Quaterniond q;
	q.x() = input.orientation.x;
	q.y() = input.orientation.y;
	q.z() = input.orientation.z;
	q.w() = input.orientation.w;
	R = q.toRotationMatrix();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh;

	ros::Subscriber rc_sub = nh.subscribe("rc_input",1,setpointCallback);
	force_pub = nh.advertise<geometry_msgs::Vector3Stamped>("force_sp",0);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);

	ros::spin();
}