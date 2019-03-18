/*
 * To run the server initiate a ros core and then run
 * rosrun rosserial_python serial_node.py tcp
 */
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "config.h"

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

// Set true to start in Access Point mode
// or false to connect to an existing wifi
#define AP_MODE_ON true

const char *ssid     = WIFI_SSID;
const char *password = WIFI_PASSWORD;  

// Set the rosserial socket server IP address
IPAddress server(ROS_MASTER_IP);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

//Physical Properties
const float WHEELBASE = 0.15;
const float WHEELRADIUS = 0.0325;

// Servo
Servo camera_pitch;
Servo camera_yaw;

const int camera_pitch_pin = D6;
const int camera_yaw_pin = D7;

ros::NodeHandle nh;

// MOTOR CONSTANTS AND PARAMETERS
const int motor_left_pwm = 5;
const int motor_right_pwm = 4;
const int motor_left_dir = 0;
const int motor_right_dir = 2;
const int motor_left_forward = LOW;
const int motor_left_reverse = HIGH;
const int motor_right_forward = HIGH;
const int motor_right_reverse = LOW;

void setup_pins() {
  // Motor pins
  pinMode(motor_left_pwm, OUTPUT);
  pinMode(motor_left_dir, OUTPUT);
  pinMode(motor_right_pwm, OUTPUT);
  pinMode(motor_right_dir, OUTPUT);
}

void setup_servos() {
  camera_pitch.attach(camera_pitch_pin);
  camera_yaw.attach(camera_yaw_pin);
}

ros::Subscriber<std_msgs::UInt16> sub_yaw("/eren/camera_yaw", drive_camera_yaw);
ros::Subscriber<std_msgs::UInt16> sub_pitch("/eren/camera_pitch", drive_camera_pitch);
ros::Subscriber<geometry_msgs::Twist> sub_twist("/eren/cmd_vel", kinematics);
ros::Subscriber<sensor_msgs::Joy> sub_joy("/eren/joy_teleop", joy_listener);

void setup_wifi() {
  if (AP_MODE_ON) {
    Serial.print("Configuring access point...");
    WiFi.softAP(ssid, password);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(apIP);
  } else {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // Connect the ESP8266 the the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void test_motors() {
  // Start the motors
  analogWrite(motor_left_pwm, 384);
  analogWrite(motor_right_pwm, 384);

  // Go Forwards 
  digitalWrite(motor_left_dir, motor_left_forward);
  digitalWrite(motor_right_dir, motor_right_forward);
  delay(1000);

  // Go Reverse
  digitalWrite(motor_left_dir, motor_left_reverse);
  digitalWrite(motor_right_dir, motor_right_reverse);
  delay(1000);

  // Shutdown pwm
  analogWrite(motor_left_pwm, 0);
  analogWrite(motor_right_pwm, 0);
}

void test_servos() {
  int pos;

  for (pos = 0; pos <= 180; pos += 1) {
    camera_pitch.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    camera_pitch.write(pos);
    delay(15);
  }

  for (pos = 0; pos <= 180; pos += 1) {
    camera_yaw.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    camera_yaw.write(pos);
    delay(15);
  }
}

void drive_motors(int left_motor_pwm, int right_motor_pwm, int left_motor_direction, int right_motor_direction) {
  digitalWrite(motor_left_dir, left_motor_direction);
  digitalWrite(motor_right_dir, right_motor_direction);
  
  analogWrite(motor_left_pwm, left_motor_pwm);
  analogWrite(motor_right_pwm, right_motor_pwm);
}

void drive_camera_pitch(const std_msgs::UInt16& cmd_msg){
  Serial.print("Got Camera Pitch: ");
  Serial.println(cmd_msg.data);
  //set servo angle, should be from 0-180  
  camera_pitch.write(cmd_msg.data);
}

void drive_camera_yaw(const std_msgs::UInt16& cmd_msg){
  Serial.print("Got Camera Yaw: ");
  Serial.println(cmd_msg.data);
  //set servo angle, should be from 0-180  
  camera_yaw.write(cmd_msg.data);
}

void joy_listener(const sensor_msgs::Joy& joy_msg) {

}

float get_motor_pwm(float velocity) {
  if (velocity < 0) {
    velocity = -velocity;
  }
  return 16.35 * pow(velocity / WHEELRADIUS, 1.31);
}

void kinematics(const geometry_msgs::Twist& twist_msg) {
  float velocity_left = 0;
  float velocity_right = 0;
  float left_motor_pwm = 0;
  float right_motor_pwm = 0;

  velocity_left = twist_msg.linear.x - twist_msg.angular.z * WHEELBASE / 2;
  velocity_right = twist_msg.linear.x + twist_msg.angular.z * WHEELBASE / 2;
  
  int left_motor_direction = motor_left_reverse;
  if (velocity_left >= 0) left_motor_direction = motor_left_forward;

  int right_motor_direction = motor_right_reverse;
  if (velocity_right >= 0) right_motor_direction = motor_right_forward;

  left_motor_pwm = get_motor_pwm(velocity_left);
  right_motor_pwm = get_motor_pwm(velocity_right);

  if (left_motor_pwm > 1023) left_motor_pwm = 1023;
  if (right_motor_pwm > 1023) right_motor_pwm = 1023;
  
  drive_motors((int) left_motor_pwm, (int) right_motor_pwm, left_motor_direction, right_motor_direction);
}

void setup() {
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  
  setup_pins();
  setup_servos();
  setup_wifi();
  
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(sub_yaw);
  nh.subscribe(sub_pitch);
  nh.subscribe(sub_twist);
  nh.subscribe(sub_joy);
}

void loop() {
  nh.spinOnce();
  
  // Loop exproximativly at 1Hz
  delay(10);
}
