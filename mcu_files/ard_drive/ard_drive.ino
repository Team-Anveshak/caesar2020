/* rosserial Subscriber For Locomotion Control */
#include <ros.h>
#include <traversal/WheelRpm.h>

#include <Wire.h>
#include <Servo.h>

#define b1 15
#define b2 16
#define b3 17
#define b4 20

int vel = 0, omega = 0;
bool hb = false;

ros::NodeHandle nh;

void loco(int address)
{
  Wire.beginTransmission(address);
  Wire.write(byte(vel));
  Wire.write(byte(vel>>8));
  Wire.write(byte(omega));
  Wire.write(byte(omega>>8));
  Wire.write(byte(hb));
  Wire.endTransmission();
}

void roverMotionCallback(const traversal::WheelRpm& RoverRpm)
{
  vel = int(RoverRpm.vel);
  omega = int(RoverRpm.omega);
  hb = RoverRpm.hb;

  loco(b1);
  loco(b2);
  loco(b3);
  loco(b4);
}


ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);

void setup() {
  nh.initNode();
  nh.subscribe(locomotion_sub);

  Wire.begin();
}

void loop() {
  nh.spinOnce();

}
