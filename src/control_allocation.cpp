#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>
#include <omnicopter_ros/MotorCommand.h>
#include <algorithm>

/*
Control allocation based on
flyingmachinearena.org/wp-content/publications/2016/breIEEE16.pdf

Input: force [N] and torque [Nm] in body frame
Output: Motor speeds [-1,1]

	 			| 1 -1  1 -1  1 -1  1 -1 |
P = d/sqrt(3) *	| 1  1 -1 -1  1  1 -1 -1 |
	 			| 1  1  1  1 -1 -1 -1 -1 |

d/sqrt(3) is the side length of a cube with diagonal distance d

	|-a  b -b  a  a -b  b -a |
X = | b  a -a -b -b -a  a  b |
	| c -c -c  c  c -c -c  c |

a = 1/2 +1/sqrt(12)
b = 1/2 -1/sqrt(12)
c = 1/sqrt(3)

	|   X	|
M = |		|
	| P x X |

*/

#define K_F 4.2e-7
#define K_M 4.2e-8 //wild guess
#define max_rpm 29000

Eigen::Vector3d force_setpoint;
Eigen::Vector3d torque_setpoint;
bool force_valid = false, torque_valid = false;
Eigen::MatrixXd X(3,8);
Eigen::MatrixXd P(3,8);
Eigen::MatrixXd M(6,8);
Eigen::MatrixXd M_pinv(6,8);

ros::Publisher motor_pub;

double motor_force_to_usec(double f){
	//std::min(std::max(x, 1000), 2000)
	if(f < 0){
		return std::min(std::max(-500/(2*M_PI*max_rpm/60.0)*sqrt(-f/K_F)+1500, 1000.0), 2000.0);
	}
	
	return std::min(std::max(500/(2*M_PI*max_rpm/60.0)*sqrt(f/K_F)+1500, 1000.0), 2000.0);
}

void forceCallback(const geometry_msgs::Vector3Stamped& input) {
	force_setpoint(0) = input.vector.x;
	force_setpoint(1) = input.vector.y;
	force_setpoint(2) = input.vector.z;
	if(!force_valid){
		force_valid = true;
	}
	Eigen::VectorXd wrench(6);
	wrench = Eigen::VectorXd::Zero(6);
	wrench.block<3,1>(0,0) = force_setpoint;
	if(torque_valid){
		wrench.block<3,1>(3,0) = torque_setpoint;
	}

	Eigen::VectorXd motor_forces(6);
	motor_forces = M_pinv*wrench;

	omnicopter_ros::MotorCommand msg;
	msg.header.stamp = ros::Time::now();
	msg.motor1_usec = motor_force_to_usec(motor_forces(0));
	msg.motor2_usec = motor_force_to_usec(motor_forces(1));
	msg.motor3_usec = motor_force_to_usec(motor_forces(2));
	msg.motor4_usec = motor_force_to_usec(motor_forces(3));
	msg.motor5_usec = motor_force_to_usec(motor_forces(4));
	msg.motor6_usec = motor_force_to_usec(motor_forces(5));
	msg.motor7_usec = motor_force_to_usec(motor_forces(6));
	msg.motor8_usec = motor_force_to_usec(motor_forces(7));

	motor_pub.publish(msg);


}

void torqueCallback(const geometry_msgs::Vector3Stamped& input) {
	
	torque_setpoint(0) = input.vector.x;
	torque_setpoint(1) = input.vector.y;
	torque_setpoint(2) = input.vector.z;
	if(!torque_valid){
		torque_valid = true;
	}
	Eigen::VectorXd wrench(6);
	wrench = Eigen::VectorXd::Zero(6);
	wrench.block<3,1>(3,0) = torque_setpoint;
	if(force_valid){
		wrench.block<3,1>(0,0) = force_setpoint;
	}

	Eigen::VectorXd motor_forces(6);
	motor_forces = M_pinv*wrench;

	omnicopter_ros::MotorCommand msg;
	msg.header.stamp = ros::Time::now();
	msg.motor1_usec = motor_force_to_usec(motor_forces(0));
	msg.motor2_usec = motor_force_to_usec(motor_forces(1));
	msg.motor3_usec = motor_force_to_usec(motor_forces(2));
	msg.motor4_usec = motor_force_to_usec(motor_forces(3));
	msg.motor5_usec = motor_force_to_usec(motor_forces(4));
	msg.motor6_usec = motor_force_to_usec(motor_forces(5));
	msg.motor7_usec = motor_force_to_usec(motor_forces(6));
	msg.motor8_usec = motor_force_to_usec(motor_forces(7));
	motor_pub.publish(msg);
	

}





int main(int argc, char **argv){
	ros::init(argc, argv, "control_allocation");
	ros::NodeHandle nh;

	// Subscribe to output from position and attitude controller
	// The torque topic will probably publish at a higher rate than the force topic
	ros::Subscriber force_sub = nh.subscribe("force_sp", 1, forceCallback);
	ros::Subscriber torque_sub = nh.subscribe("torque_sp", 1, torqueCallback);

	motor_pub = nh.advertise<omnicopter_ros::MotorCommand>("motor_commands",0);

	double a = 0.5 + 1.0/sqrt(12);
	double b = 0.5- 1.0/sqrt(12);
	double c = 1.0/sqrt(3);
	
	X << -a, b,-b, a, a,-b, b,-a,
		  b, a,-a,-b,-b,-a, a, b,
		  c,-c,-c, c, c,-c,-c, c;

	P << 1,-1, 1,-1, 1,-1, 1,-1,
		 1, 1,-1,-1, 1, 1,-1,-1,
		 1, 1, 1, 1,-1,-1,-1,-1;

	P *= 1.0/sqrt(3);

	Eigen::MatrixXd P_cross_X(3,8);
	for (int i = 0; i < 8; i++){
		Eigen::Vector3d P_col = P.col(i);
		Eigen::Vector3d X_col = X.col(i);
		P_cross_X.col(i) = P_col.cross(X_col);
	}


	M.block<3,8>(0,0) = X;
	M.block<3,8>(3,0) = P_cross_X;
	
	M_pinv = M.transpose()*((M*M.transpose()).inverse());
	

	ros::spin();

}