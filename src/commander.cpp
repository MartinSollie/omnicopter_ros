#include <ros/ros.h>

/*
Responsibility of commander:
- Arm/disarm
- Failsafe (loss of RC etc.)
*/

int main(int argc, char **argv){
	ros::init(argc, argv, "commander");
}