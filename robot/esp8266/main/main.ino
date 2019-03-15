#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include "config.h"

// Servo
Servo camera_pitch;
Servo camera_yaw;

const int camera_pitch_pin = D6;
const int camera_yaw_pin = D7;

// WIFI PARAMETERS
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// UDP SERVER PARAMETERS
WiFiUDP Udp;
const unsigned int localUdpPort = 4210;
const char STATUS_STRING[] = "status";
char incomingPacket[255];

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

void setup_wifi() {
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

void drive_servos(int re_camera_yaw, int re_camera_pitch) {
  camera_yaw.write(re_camera_yaw);
  camera_pitch.write(re_camera_pitch);
}

void setup() {
  // Open Serial port
  Serial.begin(115200);

  setup_pins();

  // Servo
  camera_pitch.attach(camera_pitch_pin);
  camera_yaw.attach(camera_yaw_pin);
  
  setup_wifi();

  Udp.begin(localUdpPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    
    if (incomingPacket[0] != STATUS_STRING[0]) {
      char *token = strtok(incomingPacket, "-"); // tokenize the string using colons as the delimiters
  
      int re_left_motor_pwm = atoi(token);
      token = strtok(NULL, "-");
      int re_right_motor_pwm = atoi(token);
      token = strtok(NULL, "-");
      int re_left_motor_dir = atoi(token);
      token = strtok(NULL, "-");
      int re_right_motor_dir = atoi(token);
      token = strtok(NULL, "-");
      int re_camera_yaw = atoi(token);
      token = strtok(NULL, "-");
      int re_camera_pitch = atoi(token);
      token = strtok(NULL, "-");
  
      drive_motors(re_left_motor_pwm, re_right_motor_pwm, re_left_motor_dir, re_right_motor_dir);
      drive_servos(re_camera_yaw, re_camera_pitch);
    } else {
      Udp.beginPacket(Udp.remoteIP(), localUdpPort);
      String str = String(analogRead(motor_left_pwm))+"-"+String(analogRead(motor_right_pwm));
      char replyPacket[str.length()+1];
      strcpy(replyPacket, str.c_str());
      
      Serial.println(replyPacket);
      Udp.write(replyPacket);
      Udp.endPacket();
    }
  }
  delay(10);
}
