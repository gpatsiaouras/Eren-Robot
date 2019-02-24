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

const char* ssid     = "Ziggo9EE38EB";
const char* password = "ujjx2atrendT";

// Set the rosserial socket server IP address
IPAddress server(192,168,178,52);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  Serial.print("Got message: ");
  Serial.println(cmd_msg.data);
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

ros::Subscriber<std_msgs::UInt16> sub("/servo", servo_cb);

void setup_wifi() {
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
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

void setup() {
  setup_wifi();
  
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(D6);
}

void loop() {
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(1000);
}
