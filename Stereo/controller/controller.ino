#include "ros.h"
#include "std_msgs/Float32.h"
#include <Servo.h>

Servo yaw_servo; // yaw servo
const int YAW_PIN = 5;
float yaw_speed = 90; // neutral

float dir = 0.0;

void cb(const std_msgs::Float32& msg) {
  dir = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32> sub("move_dir", cb);

void setup() {
  yaw_servo.attach(YAW_PIN);
  nh.initNode();
  nh.subscribe(sub);
}
int cnt = 0;

void loop() {
  nh.spinOnce();
  yaw_servo.writeMicroseconds(1500 + (100 * dir));
  delay(10);
}
