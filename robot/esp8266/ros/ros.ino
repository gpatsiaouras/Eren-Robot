/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to a Wifi Access Point
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

const char* ssid     = "Ziggo9EE38EB";
const char* password = "ujjx2atrendT";

// Servo
Servo camera_pitch;
Servo camera_yaw;

const int camera_pitch_pin = D6;
const int camera_yaw_pin = D7;

// Set the rosserial socket server IP address
IPAddress server(192,168,178,52);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

// MOTOR CONSTANTS AND PARAMETERS
const int motor_left_pwm = 5;
const int motor_right_pwm = 4;
const int motor_left_dir = 0;
const int motor_right_dir = 2;
const int motor_left_forward = LOW;
const int motor_right_forward = HIGH;
const int motor_left_reverse = HIGH;
const int motor_right_reverse = LOW;

void setup_pins() {
  // Motor pins
  pinMode(motor_left_pwm, OUTPUT);
  pinMode(motor_left_dir, OUTPUT);
  pinMode(motor_right_pwm, OUTPUT);
  pinMode(motor_right_dir, OUTPUT);
}

ros::Subscriber<std_msgs::UInt16> sub_yaw("/eren/camera_yaw", drive_camera_yaw);
ros::Subscriber<std_msgs::UInt16> sub_pitch("/eren/camera_pitch", drive_camera_pitch);
ros::Subscriber<geometry_msgs::Twist> sub_twist("/eren/cmd_vel", kinematics);

void setup_wifi() {
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

void test_motors() {
  // Set pwm in 1/4 of full power
  analogWrite(motor_left_pwm, 256);
  analogWrite(motor_right_pwm, 256);

  // Go Forwards 
  digitalWrite(motor_left_dir, motor_left_forward);
  digitalWrite(motor_right_dir, motor_right_forward);
  delay(2000);

  // Go Reverse
  digitalWrite(motor_left_dir, motor_left_reverse);
  digitalWrite(motor_right_dir, motor_right_reverse);
  delay(2000);

  // Shutdown pwm
  analogWrite(motor_left_pwm, 0);
  analogWrite(motor_right_pwm, 0);
}

void test_servos() {
  int pos;

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    camera_pitch.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    camera_pitch.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    camera_yaw.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    camera_yaw.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void drive_motors(int re_left_motor_pwm, int re_right_motor_pwm, int re_left_motor_dir, int re_right_motor_dir) {
  if (re_left_motor_dir == 1) {
    digitalWrite(motor_left_dir, motor_left_forward);  
  } else {
    digitalWrite(motor_left_dir, motor_left_reverse);
  }
  if (re_right_motor_dir == 1) {
    digitalWrite(motor_right_dir, motor_right_forward);   
  } else {
    digitalWrite(motor_right_dir, motor_right_reverse);   
  }
  
  analogWrite(motor_left_pwm, re_left_motor_pwm);
  analogWrite(motor_right_pwm, re_right_motor_pwm);
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

void kinematics(const geometry_msgs::Twist& twist_msg) {
  Serial.print("Linear: ");
  Serial.println(twist_msg.linear.x);
  Serial.print("Angular: ");
  Serial.println(twist_msg.angular.z);
}

void setup() {
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  setup_pins();
  
  // Servo
  camera_pitch.attach(camera_pitch_pin);
  camera_yaw.attach(camera_yaw_pin);
  
  setup_wifi();
  
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(sub_yaw);
  nh.subscribe(sub_pitch);
}

void loop() {
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(10);
}
