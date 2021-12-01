# CABA

Enviroment Analysis and Notifying Vehicle (turkish shortening: **CABA**) is a 4 wheeled vehicle which its goal is helping to the police and the other law enforcers. 
It has 55 cm length and 35 cm width. It's produced from sheet metal and the weight is aproximately 9.5 kilograms. Being both light and small, it can be used in any area and can be carried easily 
to any area. The activation of CABA will help the law enforcement officers to develop tactics. In addition, in order to prevent possible loss of life, it will be an advantage for the law enforcement 
officers if CABA enters the area that must or cannot be entered before the law enforcement officers and fulfills their duties.
  
Robot Operating System (ROS) is being used for autonomous driving. Besides it has Raspberry Pi Camera v2 in order to detect humans. In order to analyze the enviroment, ÇABA has 4 sensors. These sensors are:
  
* DfRobot Analog Ambient Light Sensor,                                                                                                                                    
* MQ-135 Gas Sensor,                                                                                                                                                          
* STH75 Temperature and Humidity Sensor,                                                                                                                                              
* Flame Detecting Sensor                                                                                                                                              
  
![CABA](https://github.com/zuslok/CABA/blob/4bab5570388d7fbaaa1ad60f2e4eb50bfaff1dea/caba_frontleft.jpeg)
  
 ***This repositery is only about the ROS part of the CABA. Computer Vision and Sensors aren't explaining in this repositery.***
 
 ## Requirements 
 
 * [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)                                                                                                                   
 * [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)                                                                                                           
 * Some ROS packages:
 ```bash
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-slam-gmapping
sudo apt-get install ros-melodic-amcl
```
  - [ROS-Melodic Laser Scan Matcher](https://github.com/ccny-ros-pkg/scan_tools)
  - [ROS Arduino Serial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
  - [RPLidar package](http://wiki.ros.org/rplidar)

## Hardware

In this section will be explained which hardware was used for CABA and how was assembled.

- [Jetson Nano 4GB Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
- [RPLidar A1M8](https://www.slamtec.com/en/Lidar/A1)
- [Arduino Mega](https://store.arduino.cc/products/arduino-mega-2560-rev3)
- [tp-link Wifi Adapter](https://www.tp-link.com/en/home-networking/adapter/tl-wn725n/)
- 2x [Pololu PL-2825](https://www.pololu.com/product/2825)
- 2x [PL80 wheel](https://www.direnc.net/pl80-tekerlek-mor)
- 2x Caster Wheel
- 2x BTS7960 40 Amper Motor Driver
- 2200 mAh Li-Ion Battery
- Voltage Dropper to 5V
- Battery Charger
- On/Off Button

![Hardware](https://github.com/zuslok/CABA/blob/4bab5570388d7fbaaa1ad60f2e4eb50bfaff1dea/caba_inside.jpeg)

- BTS7960 is the most optimal motor driver for encoder motors. As a cheaper option, the L298N can be used. Please click this [link](https://www.youtube.com/watch?v=ZlteJi6rsd0&t=10s)
for BTS7960 and Arduino connection. Communication between Arduino Mega and Jetson takes place via the rosserial package. ROS topics are shared between Arduino and Jetson.
You can check out the Arduino code in ***CABA/Arduino/DC_Motor_Control/DC_Motor_Control.ino*** and you can see PINOUTS for the motor drivers and encoders.

- Voltage Dropper was used for Jetson Nano. The battery suplies 12V and Jetson Nano's optimum operating voltage is 5V. For this reason dropping voltage is necessary.

# How to Use CABA

In order to use CABA your Jetson and your master computer need to connect the network.

## 1. Connect to Jetson via SSH
Open the Linux terminal on your machine and run the following command: 
 ```bash
ssh your_host_username@host_ip_address 
 ```
## 2. Open Motor Control and Lidar
After connecting Jetson's terminal you need to open the motors and the laser scanner. Run the following command on host terminal:
 ```bash
roslaunch caba_bringup caba_bringup.launch
 ```
Thus it will launch the serial controller node connected to the Arduino, odometry topic is activated, robot starts to receive scan datas from RPLidar and loads the joint state publisher. 

***CABA is ready to go!***

