/* rosserial Subscriber For Locomotion Control */
#include <ros.h>
#include <traversal/WheelRpm.h>
#include <sensors/PanTilt.h>

#include <Wire.h>
#include <Servo.h>

#define b1 15
#define b2 16
#define b3 17
#define b4 20

Servo pan;
Servo tilt;

int vel = 0, omega = 0;
bool hb = false;
int panAngle = 0;
int tiltAngle = 0;
int pan_pos = 10, tilt_pos = 10;
int servo_then;

ros::NodeHandle nh;

void loco(int address)
{
  Wire.beginTransmission(address);
  Wire.write(byte(vel));
  //Wire.write(byte(vel>>8)); byte shifting disabled for old stm new stm problem
  Wire.write(byte(omega));
  //Wire.write(byte(omega>>8));
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

void servoCallback(const sensors::PanTilt& Control)
{
  panAngle = constrain(Control.pan,1,170);
  tiltAngle = constrain(Control.tilt,5,150);
  
  pan.write(panAngle);
  tilt.write(tiltAngle);

}

ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);
ros::Subscriber<sensors::PanTilt> pantilt_sub("pan_tilt_ctrl", &servoCallback);

void setup() {
  nh.initNode();
  nh.subscribe(locomotion_sub);
  nh.subscribe(pantilt_sub);
  Wire.begin();
  pan.attach(10);
  tilt.attach(11);
}

void loop() {
  nh.spinOnce();

}
