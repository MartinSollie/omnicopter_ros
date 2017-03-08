/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Name        : I2C.cpp
 * Author      : Georgi Todorov
 * Version     :
 * Created on  : Dec 30, 2012
 *
 * Copyright © 2012 Georgi Todorov  <terahz@geodar.com>
 */

#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <linux/i2c.h>
#include <syslog.h>		/* Syslog functionality */
#include "i2c.h"

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port


unsigned char buffer[60] = {0};

int main(void){
	printf("Initialize i2c\n", );
	i2c_init(70);
	while (1) {
		i2c_read_byte(1,buffer);
		printf("read: %d\n", buffer);
	}
}
void i2c_init(int address){
	int file_i2c;
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return;
	}

	if (ioctl(file_i2c, I2C_SLAVE, address) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
}

void i2c_read_byte(int length, unsigned char buffer){
	if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Data read: %s\n", buffer);
	}

}

void i2c_write_byte(int address, int length){
	buffer[0] = 0x01;
	buffer[1] = 0x02;
	if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
	}
}
