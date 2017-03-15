# omnicopter_ros

How to setup a Raspberry Pi Zero to run the omnicopter software:

(WIP)

1. Install Raspbian Jessie Lite by following the instructions here: https://www.raspberrypi.org/documentation/installation/installing-images/

2. Connect the Pi Zero to a monitor using the mini HDMI connector. Use a USB OTG adapter to connect a keyboard to the single USB port of the Pi Zero. Power the Pi Zero up by connecting a USB power adapter that can deliver minimum 1A to the "PWR IN" USB port. Default login user "pi", password "raspberry".

3. Enter `sudo raspi-config`. The hostname and password can be changed here if desired (options 1 and 2). Select "5 Interfacing Options" and then enable SSH (P2) and I2C (P5).

The remaining part of the setup can be done over SSH. If you do not use SSH you will probably need a USB hub to connect to the internet while still having the keyboard connected. You might also want to change the keyboard layout if you are not using a UK keyboard layout.

4. Connect the Pi to the internet using a USB-Ethernet adapter, wifi dongle or shared internet from a Smartphone

5. Increase the amount of swap space from 100MB to 500MB. 100MB is a bit to low for compiling ROS: open `/etc/dphys-swapfile` using e.g. vim or nano and change `CONF_SWAPSIZE=100` to `CONF_SWAPSIZE=500`. Then restart the swap service:
```
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
```


6. Clone this repo and run the startup script. This may take a long time (hours) to complete since we will be compiling ROS from source:
```
mkdir -p ~/omnicopter_ws/src
cd ~/omnicopter_ws/src
git clone https://github.com/MartinSollie/omnicopter_ros.git
cd omnicopter_ros
chmod +c startup.sh
sudo ./startup.sh
```




Wifi: https://oshlab.com/esp8266-raspberry-pi-gpio-wifi/

Installere ROS: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
http://wiki.ros.org/indigo/Installation/UbuntuARM#Set_your_Locale

Installere RTIMULib: https://github.com/RPi-Distro/RTIMULib/tree/master/Linux

Last ned Eigen herfra: http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
Pakk ut filene og kopier mappa "Eigen" til /usr/local/include

