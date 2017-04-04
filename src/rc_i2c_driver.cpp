#include <ros/ros.h>
#include <omnicopter_ros/RCInput.h>
#include <omnicopter_ros/I2C.h>
#include <cmath>

#define I2C_SLAVE 0x50
#define MAX 1811
#define MIN 172

int main(int argc, char **argv){
	ros::init(argc, argv, "rc_i2c_driver");
 	ros::NodeHandle nh("~");

	ros::Publisher pub_rc = nh.advertise<omnicopter_ros::RCInput>("rc_input", 1);

	//Initialize I2C RC stuff
	I2C rc_i2c(1, I2C_SLAVE);

	uint8_t data[14];
	uint16_t byte_ch;
	uint8_t byte_h, byte_l;
	float ch[7];

	ros::Rate loop_rate(50);
	omnicopter_ros::RCInput msg;

	while(ros::ok()){
		//Read 7 channels (14 bytes)
		if(rc_i2c.read_block(0, data, 14)){
			for (int i = 0; i < 7; i++){
				byte_h = data[2*i];
				byte_l = data[2*i+1];
				byte_ch = byte_l | byte_h << 8;
				ch[i] = 2 * ((byte_ch - MIN)/(float)(MAX - MIN)) - 1; //Normalizing data
			}

			msg.header.stamp = ros::Time::now();
			msg.rollstick = ch[0];
			msg.pitchstick = ch[1];
			msg.throttlestick = ch[2];
			msg.yawstick = ch[3];
			msg.arm = (bool)round(ch[4]);
			
			int mode1 = round(ch[5]);
			int mode2 = round(ch[6]);
			if(mode1 == 1 && mode2 == 1){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_RP_ATT_Y_RATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_BODY_UP;
			}
			else if(mode1 == 1 && mode2 == 0){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_YAWRATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_BODY;

			}
			else if(mode1 == 1 && mode2 == -1){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_YAWRATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_ENU;
			}
			else if(mode1 == 0 && mode2 == 1){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_RP_ATT_Y_RATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_ALTHOLD_FORCE_BODY_UP;
			}
			else if(mode1 == 0 && mode2 == 0){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_YAWRATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_ALTHOLD_FORCE_BODY;
			}
			else if(mode1 == 0 && mode2 == -1){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_YAWRATE;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_ALTHOLD_FORCE_ENU;
			}
			else if(mode1 == -1){
				msg.rc_mode.attitude_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_RATES;
				msg.rc_mode.position_control_mode = omnicopter_ros::ControlMode::MODE_CONTROL_FORCE_ENU;
			}
			pub_rc.publish(msg);

		}
		loop_rate.sleep();
	}
}
