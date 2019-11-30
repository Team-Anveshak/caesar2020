// NOTE all outputs are ACTIVE LOW

HardwareTimer timer(3);

#include <stdlib.h>

#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS_PIN PA4
MCP_CAN CAN (SPI_CS_PIN);


#define LED PC13

#define PWM1 PA9
#define INB1 PB8
#define INA1 PB7
#define EN1  PB0

#define PWM2 PA8
#define INB2 PB6
#define INA2 PA3
#define EN2  PA2

#define PWMMAX 0xFFFF

void initAndClearOutputs ();
void fixPWM ();
void initCAN ();
void updatePinBuff ();
void writeMotors ();

// for CAN
#define STD_FRAME 0
const   unsigned long SELF_ID = 0xF2;

#define LOST_COMM_INTERVAL 200
unsigned long last_comm_time = 0;
unsigned long curr_time = 0;

// PWM, EN, INA, INB (in that order)
uint8_t buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup () {
  fixPWM ();
  writeMotors();
  initAndClearOutputs ();
  initCAN();

  curr_time = millis();
  last_comm_time = curr_time;
}

void loop () {
  curr_time = millis();

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    unsigned char len;
    unsigned char tmp[8];
    CAN.readMsgBuf (&len, tmp);
    if (CAN.getCanId() == SELF_ID)
    {
      for (int i = 0; i < 8; i++)
        buff[i] = tmp[i];
      last_comm_time = curr_time;
      digitalWrite (LED, HIGH);
    }
  }
  
  if (curr_time - last_comm_time > LOST_COMM_INTERVAL)
  {
    for (int i = 0; i < 8; i++)
      buff[i] = 0;
    last_comm_time = curr_time;
    digitalWrite (LED, LOW);
  }

  writeMotors();
}




void writeMotors () {
  // motor_buff in this order: pwm, en, ina, inb
  pwmWrite (PWM1,     buff[0]*PWMMAX/0xFF);
  digitalWrite (EN1,  buff[1]);
  digitalWrite (INA1, buff[2]);
  digitalWrite (INB1, buff[3]);
  pwmWrite (PWM2,     buff[4]*PWMMAX/0xFF);
  digitalWrite (EN2,  buff[5]);
  digitalWrite (INA2, buff[6]);
  digitalWrite (INB2, buff[7]);
}

void fixPWM() {
  timer.setPrescaleFactor(4);
  timer.setOverflow(1024);
  timer.refresh();
}

void initCAN () {
  Serial.begin(115200);
  digitalWrite (LED, LOW);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
      digitalWrite (LED, HIGH);
      delay (500);
      digitalWrite (LED, LOW);
      delay (500);
  }
  
  digitalWrite (LED, HIGH);
}

void initAndClearOutputs () {
  pinMode (LED, OUTPUT);
  pinMode (PWM1, PWM);
  pinMode (INB1, OUTPUT);
  pinMode (INA1, OUTPUT);
  pinMode (EN1, OUTPUT);
  pinMode (PWM2, PWM);
  pinMode (INB2, OUTPUT);
  pinMode (INA2, OUTPUT);
  pinMode (EN2, OUTPUT);

  digitalWrite (LED, HIGH);
  pwmWrite (PWM1, PWMMAX);
  digitalWrite (INB1, HIGH);
  digitalWrite (INA1, HIGH);
  digitalWrite (EN1, HIGH);
  pwmWrite (PWM2, PWMMAX);
  digitalWrite (INB2, HIGH);
  digitalWrite (INA2, HIGH);
  digitalWrite (EN2, HIGH);
}
