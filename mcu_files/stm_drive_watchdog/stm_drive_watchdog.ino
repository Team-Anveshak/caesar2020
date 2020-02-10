#include <IWatchdog.h>
#include <Wire.h>

/////
int address = 16;   //change address here 15,16,17
/////

#define CSr PA2
#define CSl PA3
#define slpr PB14
#define slpl PA11
#define DIRr PA10
#define DIRl PB15
#define PWMr PA8
#define PWMl PA9


int vel1 ,vel2 , om1 ,om2 = 0, hb =0;
int pwmr = 0, pwml = 0;
int vel,omega=0;

void receiveEvent(int howMany)
{
  IWatchdog.reload();
  
  if ( Wire.available())
  {
    vel1 = Wire.read();
    //vel2 = Wire.read();
    om1 = Wire.read();
    //om2 = Wire.read();
    hb = Wire.read();
    
    //vel = vel1 + (vel2<<8);
    //omega = om1 + (om2<<8);
    
    vel=vel1-127;
    omega=om1-127;	  
	
  }
}

void setup()
{ 
  //DEBUG
  pinMode (PC13, OUTPUT);
  digitalWrite (PC13, HIGH);
  delay (1000);
  digitalWrite (PC13, LOW);

  IWatchdog.begin(5000000);
  
  Serial.end();
  pinMode(PA11, OUTPUT);
  //timer.setPrescaleFactor(32);
  //timer.setOverflow(127);
  //timer.refresh();

  pinMode(PWMl, OUTPUT);
  pinMode(PWMr, OUTPUT);
  pinMode(slpl, OUTPUT);
  pinMode(slpr, OUTPUT);
  pinMode(DIRr, OUTPUT);
  pinMode(DIRl, OUTPUT);
  pinMode(CSr, INPUT);
  pinMode(CSl, INPUT);

  digitalWrite(slpl, HIGH);
  digitalWrite(slpr, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin(address);

  Wire.onReceive(receiveEvent);

 //pwmWrite(PWMl, 0);
  //pwmWrite(PWMr,0);
  digitalWrite(slpl, LOW);
  digitalWrite(slpr,LOW);
  digitalWrite(DIRr, LOW);
  digitalWrite(DIRl, LOW);

}

void loop()
{
  analogWriteFrequency(17000);
  pwmr = int(vel+omega);
  pwml = int(vel-omega);

  if (hb==0)
  {
	  if (pwmr > 0)
	  {
	    digitalWrite(slpr, HIGH);
	    digitalWrite(DIRr, HIGH);
	    analogWrite(PWMr, abs(pwmr));
	  }
	  else if (pwmr < 0)
	  {
	    digitalWrite(slpr, HIGH);
	    digitalWrite(DIRr, LOW);
	    analogWrite(PWMr, abs(pwmr));
	  }
	  else
	  {
	    digitalWrite(slpr, LOW);
	    analogWrite(PWMr, 0);
	  }

	  if (pwml > 0)
	  {
	    digitalWrite(slpl, HIGH);
	    digitalWrite(DIRl, LOW);
	    analogWrite(PWMl, abs(pwml));
	  }
	  else if (pwml < 0)
	  {
	    digitalWrite(slpl, HIGH);
	    digitalWrite(DIRl, HIGH);
	    analogWrite(PWMl, abs(pwml));
	  }
	  else
	  {
	    digitalWrite(slpl, LOW);
	    analogWrite(PWMl, 0);
	  }
  }
  else
  {
  	digitalWrite(slpr, HIGH);
	analogWrite(PWMr, 0);
	digitalWrite(slpl, HIGH);
	analogWrite(PWMl, 0);
  }

  delay (10);

}
