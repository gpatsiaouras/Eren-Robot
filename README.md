# Eren Robot
## Introduction
Eren robot is a 4wd robot used as a base to run Artificial Intelligence algorithms.
## How to use
The robot code is implemented in two different ways. First way is with udp packages only and the second is by using the ros framework.
For both ways you need to create a ```config.h``` located next to main.ino or/and ros.ino defining your WIFI_SSID, WIFI_PASSWORD, and ROS_MASTER_IP like this:
```
#define WIFI_SSID "my_wifi_ssid"
#define WIFI_PASSWORD "my_wifi_password"
#define ROS_MASTER_IP 192,168,178,52 # Not needed if not using ros
```
## How to run
### UDP Method
To run the first and simple methods with udp packages just upload the main.ino file to esp8266 using the Arduino IDE and then send udp packages with dash seperated values like this:
```
1023-1023-1-1-150-150
LeftPwm-RightPwm-LeftDirection-RightDirection-CameraYaw-CameraPitch
```
To test in linux you can run this command:
```
$ echo "512-512-1-1-120-120" > /dev/udp/<robot-ip-addr>/4210
```
### ROS Method (Melodic)
To run with ros you will have to open a roscore and then run the command:
```
$ roscore
$ rosrun rosserial_python serial_node.py tcp
```
If you are missing packages run ```sudo apt install ros-melodic-rosserial ros-melodic-rosserial-server ros-melodic-rosserial-python```
#### Alternatively run using docker
```
$ docker run --rm --name ros-eren-serial -t --port 11311:11311 --port 11411:11411 gpatsiaouras/ros-eren
```

Kill docker container when done by running: ```docker rm -f ros-eren-serial```