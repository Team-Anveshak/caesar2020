#include <Ethernet.h>
#include <EthernetUdp.h>
#include<string.h>
#include <Wire.h>


// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address
IPAddress server(192,168,0,2);
IPAddress client(192,168,0,5);
int port  = 10006;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

//create UDP instance
EthernetUDP udp;

#define b1 15
#define b2 16
#define b3 17
#define b4 20

int vel = 0, omega = 0;
int hb = 0;
String c;
char* vel_;
char* omega_;
char* hb_;

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


void setup()
{
  //Serial.begin(19200);
  Ethernet.begin(mac,client);
  Wire.begin();
  while (Ethernet.linkStatus() ==LinkOFF) 
  {
   delay(500);
  }
  udp.begin(port);
}

void loop()
{
  uint8_t buffer[50] = "hello world";
  udp.beginPacket(server, port);
  udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, 50);
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 50) > 0)
  {
    //Serial.println((char *)buffer);
    c= (char*)buffer;
    int n=c.length();
    char c_[n+1];
    strcpy(c_,c.c_str());
    vel_=strtok(c_,",");
    omega_=strtok(NULL,",");
    hb_=strtok(NULL,".");
    vel=atoi(vel_);
    omega=atoi(omega_);
    hb=atoi(hb_);
    //Serial.print(c);
    //Serial.print('\n');
    //Serial.print(vel);
    //Serial.print('\t');
    //Serial.print(omega);   
    //Serial.print('\t');
    //Serial.print(hb);
    //Serial.print('\n');     
    loco(b1);
    loco(b2);
    loco(b3);
    loco(b4); 
  }
}
