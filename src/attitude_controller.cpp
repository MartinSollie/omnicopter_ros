#include <ros/ros.h>
#include <omnicopter_ros/RCInput.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <stdio.h>

#define T_ATT 0.2 // Attitude control time constant
#define T_W 0.01 // Angular rate control time constant

#define MAX_ROLL 0.7f
#define MAX_PITCH 0.7f

#define MAX_PITCHRATE_CMD 3.0f
#define MAX_ROLLRATE_CMD 3.0f
#define MAX_YAWRATE_CMD 3.0f

/*
Currently the controller only runs once when a new input is received.
The input persistence of the PWM driver means that the motor outputs corresponding
to this control iteration will be kept until the next iteration.
For safety reasons this may not be optimal, consider making changes.

TODO: fix handling of temporary failsafe (reset setpoint_received, imu_received)
*/

ros::Publisher torque_pub;
const Eigen::Matrix3d J = 0.0003*Eigen::Matrix3d::Identity(); //Inertia matrix
bool imu_received = false;
bool setpoint_received = false;
sensor_msgs::Imu imu_data;
omnicopter_ros::RCInput setp;

bool hold_attitude, hold_yaw;
geometry_msgs::Quaternion q_hold;
void doControl();
Eigen::Vector3d control_attitude(const Eigen::Quaterniond q_des, const Eigen::Quaterniond q);
Eigen::Vector3d control_rates(const Eigen::Vector3d w_des, const Eigen::Vector3d w);
float yaw_h;
Eigen::Quaterniond q_setp;
Eigen::Quaterniond q_err;

void imuCallback(const sensor_msgs::Imu& input){
	imu_data = input;
	if(!imu_received){
		Eigen::Quaterniond q_tmp;
		q_tmp.x() = input.orientation.x;
		q_tmp.y() = input.orientation.y;
		q_tmp.z() = input.orientation.z;
		q_tmp.w() = input.orientation.w;
		Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  		yaw_h = euler[2];
		imu_received = true;
	}
	if(setpoint_received){
		doControl();
	}

}

void setpointCallback(const omnicopter_ros::RCInput& input){
	setp = input;
	if(!setpoint_received){
		setpoint_received = true;
	}
	//if(imu_received){
	//	doControl();
	//}
}

Eigen::Quaterniond RPquaternionFromRC(const omnicopter_ros::RCInput& input, bool zeroRP){
	float roll = zeroRP ? 0 : MAX_ROLL*input.rollstick;
	float pitch = zeroRP ? 0 : MAX_PITCH*input.pitchstick;
	yaw_h -= input.yawstick*0.03;
	while(yaw_h > M_PI){
		yaw_h -= 2*M_PI;
	}
	while(yaw_h < -M_PI){
		yaw_h += 2*M_PI;
	}
	q_setp = Eigen::AngleAxisd(yaw_h,  Eigen::Vector3d::UnitZ())
	* Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
	* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q_setp;
}

Eigen::Vector3d ratesFromRC(const omnicopter_ros::RCInput& input){
	return Eigen::Vector3d(input.rollstick*MAX_ROLLRATE_CMD, input.pitchstick*MAX_PITCHRATE_CMD, -input.yawstick*MAX_YAWRATE_CMD);
}



void doControl(){
	Eigen::Quaterniond q_des;
	Eigen::Quaterniond q_curr;
	Eigen::Vector3d w_des;
	Eigen::Vector3d w_curr;
	q_curr.x() = imu_data.orientation.x;
	q_curr.y() = imu_data.orientation.y;
	q_curr.z() = imu_data.orientation.z;
	q_curr.w() = imu_data.orientation.w;

	/*if(setp.rc_mode.attitude_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_ATT){
		//Do attitude control
		hold_attitude = false;
		hold_yaw = false;
		q_des = RPquaternionFromRC(setp);
		

		w_des = control_attitude(q_des, q_curr);
	}
	else */if(setp.rc_mode.attitude_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_RP_ATT_Y_RATE){
		Eigen::Vector3d w_rc = ratesFromRC(setp);
		w_des = control_attitude(RPquaternionFromRC(setp, false), q_curr);
		
	}
	else if(setp.rc_mode.attitude_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_YAWRATE){
		Eigen::Vector3d w_rc = ratesFromRC(setp);
                w_des = control_attitude(RPquaternionFromRC(setp, true), q_curr);

	}
	else if(setp.rc_mode.attitude_control_mode == omnicopter_ros::ControlMode::MODE_CONTROL_RATES){
		// If the rate setpoint is ~0, we want to hold the current attitude, but only sample it when the
		// actual rate gets small so we don't get a big bounceback
		Eigen::Vector3d w_rc = ratesFromRC(setp);
		bool zero_setpoint = std::abs(w_rc(0)) < 0.001 && std::abs(w_rc(1)) < 0.001 && std::abs(w_rc(2)) < 0.001;
		bool small_rate = std::abs(imu_data.angular_velocity.x) < 0.5 && 
							std::abs(imu_data.angular_velocity.y) < 0.5 && 
							std::abs(imu_data.angular_velocity.z) < 0.5;
		if(zero_setpoint && small_rate){
			if(!hold_attitude){
				q_hold = imu_data.orientation;
				hold_attitude = true;
			}
			q_des.x() = q_hold.x;
			q_des.y() = q_hold.y;
			q_des.z() = q_hold.z;
			q_des.w() = q_hold.w;

			w_des = control_attitude(q_des, q_curr);

		}
		else{
			// Do rate control
			hold_attitude = false;
			w_des = w_rc;
			
		}
	}
	w_curr(0) = imu_data.angular_velocity.x;
	w_curr(1) = imu_data.angular_velocity.y;
	w_curr(2) = imu_data.angular_velocity.z;

	Eigen::Vector3d torque_out = control_rates(w_des, w_curr);
	geometry_msgs::Vector3Stamped msg;
	msg.vector.x = torque_out(0);
	msg.vector.y = torque_out(1);
	msg.vector.z = torque_out(2);
	torque_pub.publish(msg);
}

Eigen::Vector3d control_attitude(const Eigen::Quaterniond q_des, const Eigen::Quaterniond q){
	q_err = q.inverse()*q_des;
	// Add feedforward if we need trajectory tracking
	if (q_err.w() >= 0){
		return 2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	else {
		return -2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	
}

Eigen::Vector3d control_rates(const Eigen::Vector3d w_des, const Eigen::Vector3d w){
	return 1.0/T_W*J*(w_des-w)+w.cross(J*w);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "attitude_controller");
	ros::NodeHandle nh;

	ros::Subscriber rc_sub = nh.subscribe("rc_input",1,setpointCallback);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);
	torque_pub = nh.advertise<geometry_msgs::Vector3Stamped>("torque_sp",0);

	ros::spin();
}
