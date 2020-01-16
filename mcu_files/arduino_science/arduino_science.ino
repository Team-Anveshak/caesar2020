#include <ros.h>
#include <science/actuators.h>
#include <science/sensor.h>
#include <Wire.h>
#include "TSL2561.h"
#include <LPS.h>
#include <SHT1x.h>

#define dataPin 12
#define clockPin 13

int angle_stepper=30;
int i=0;

ros::NodeHandle nh;
science::sensor msg;

TSL2561 tsl(TSL2561_ADDR_FLOAT);
LPS ps;
SHT1x sht1x(dataPin, clockPin);
ros::Publisher pub("s_val", &msg);

void actuatorCallback(const science::actuators& obj)
{ 
  if (obj.stepper ==1)
  {
    for(i=0;i<2*angle_stepper;i++)
    {
    digitalWrite(39,HIGH);
    digitalWrite(41, HIGH- digitalRead(41));
    digitalWrite(LED_BUILTIN, HIGH- digitalRead(LED_BUILTIN));
    delay(5);
    }
  }
  if (obj.stepper==-1)
  {
    for(i=0;i<2*angle_stepper;i++)
    {
    digitalWrite(39,LOW);
    digitalWrite(41, HIGH- digitalRead(41));
    digitalWrite(LED_BUILTIN, HIGH- digitalRead(LED_BUILTIN));
    delay(5);
    }
  }
  
  if (obj.linac==0)
  {
    digitalWrite(2,LOW);
    digitalWrite(3,LOW);
  }
  if (obj.linac==1)
  {
    analogWrite(2,150);
    digitalWrite(3,LOW);
  }
  if (obj.linac==-1)
  {
    digitalWrite(2,LOW);
    analogWrite(3,150); 
  }

  
  if (obj.micro==0)
  {
    digitalWrite(5,LOW);
    digitalWrite(4,LOW);
  }
  if (obj.micro==1)
  {
    analogWrite(5,200);
    digitalWrite(4,LOW);
  }
  if (obj.micro==-1)
  {
    digitalWrite(5,LOW);
    analogWrite(4,200); 
  }

  
  if (obj.lid==0)
  {
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
  }
  if (obj.lid==1)
  {
    analogWrite(7,100);
    digitalWrite(6,LOW);
  }
  if (obj.lid==-1)
  {
    digitalWrite(7,LOW);
    analogWrite(6,100); 
  }

  
  if (obj.rhino==0)
  {
    digitalWrite(9,LOW);
    digitalWrite(8,LOW);
  }
  if (obj.rhino==1)
  {
    analogWrite(9,100);
    digitalWrite(8,LOW);
  }
  if (obj.rhino==-1)
  {
    digitalWrite(9,LOW);
    analogWrite(8,100); 
  }
  if (obj.sht==1)
  {
     msg.sht_humidity=sht1x.readHumidity();
     msg.sht_temp= sht1x.readTemperatureC();
     msg.tsl_full = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
     msg.lps_temp= ps.readTemperatureC();//sht1x.readTemperatureC();
     msg.lps_pressure=ps.readPressureMillibars();
     pub.publish(&msg);
  }
}

ros::Subscriber<science::actuators> act_sub("motion_sci", &actuatorCallback);

/*void sense()
{
  //msg.ir = tsl.getLuminosity(TSL2561_INFRARED);
  //msg.visible = tsl.getLuminosity(TSL2561_VISIBLE);
  msg.tsl_full = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
  msg.lps_temp= ps.readTemperatureC();//sht1x.readTemperatureC();
  msg.lps_pressure=ps.readPressureMillibars();
  //msg.sht_temp= 0; //sht1x.readTemperatureC();
  //msg.sht_humidity=0; //sht1x.readHumidity();
  pub.publish(&msg);
  //delay(500);
}*/

void setup()
{
  Wire.begin();
  nh.initNode();
  nh.advertise(pub);
  nh.setSpinTimeout(40000);
  nh.subscribe(act_sub);
  pinMode(39,OUTPUT);//stepper direction
  pinMode(41,OUTPUT);//stepper step
  pinMode(25,OUTPUT);//ENA-1-linac
  digitalWrite(25,HIGH);
  pinMode(23,OUTPUT);//ENB-1
  digitalWrite(23,LOW);
  pinMode(2,OUTPUT);//PWM1-1
  pinMode(3,OUTPUT);//PWM2-1
  pinMode(29,OUTPUT);//ENA-2-micro
  digitalWrite(29,HIGH);
  pinMode(27,OUTPUT);//ENB-2
  digitalWrite(27,LOW);
  pinMode(5,OUTPUT);//PWM1-2
  pinMode(4,OUTPUT);//PWM2-2
  pinMode(30,OUTPUT);//ENA-3-lid
  digitalWrite(30,HIGH);
  pinMode(32,OUTPUT);//ENB-3
  digitalWrite(32,LOW);
  pinMode(7,OUTPUT);//PWM1-3
  pinMode(6,OUTPUT);//PWM2-3
  pinMode(34,OUTPUT);//ENA-4-rhino
  digitalWrite(34,HIGH);
  pinMode(36,OUTPUT);//ENB-4
  digitalWrite(36,LOW);
  pinMode(9,OUTPUT);//PWM1-4
  pinMode(8,OUTPUT);//PWM2-4 
  if (!ps.init())
  {
    while(1);
  }
  ps.enableDefault();
  tsl.setGain(TSL2561_GAIN_16X); //TSL2561_GAIN_0X for bright situations
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS); // depeding on brightness of surroundings - 101MS or 402MS(dim surroundings) 
}

void loop()
{
  //sense();
  nh.spinOnce();
}
