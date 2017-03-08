#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "../include/omnicopter_ros/i2c.h"

int main(void){
  i2c = new I2C(1,0x70);

  while(1){
    int buf = 0;
    buf = i2c->read_byte(0x68)
    printf("IMU: %d\n", buf);
  }
}
