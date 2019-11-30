// NOTE all outputs are ACTIVE LOW

HardwareTimer timer(3);

#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

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

void callback (const std_msgs::Int32MultiArray& msg);
void initAndClearOutputs ();
void fixPWM ();
void initCAN ();
void updatePinBuff ();
void writeMotors ();
void writeCanBus ();

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub ("stm_write", callback);

// for CAN
#define STD_FRAME 0
#define ID_A 0xF1
#define ID_B 0xF2

#define WRITE_INTERVAL 20
#define LOST_COMM_INTERVAL 200
unsigned long last_write_time = 0;
unsigned long last_comm_time = 0;
unsigned long curr_time = 0;

#define INP_LEN 6
int32_t recv_buff[INP_LEN] = {0, 0, 0, 0, 0, 0};

// PWM, EN, INA, INB (in that order)
uint8_t pin_buff[INP_LEN * 4];
uint8_t *motor_buff = pin_buff;
//uint8_t *can_buff = pin_buff + 8;

void setup () {
  fixPWM ();
  initAndClearOutputs ();
  updatePinBuff();
  writeMotors();
  initCAN();
  writeCanBus();

  curr_time = millis();
  last_write_time = curr_time;
  last_comm_time = curr_time;
  
  nh.initNode ();
  nh.subscribe (sub);
}

void loop () {
  curr_time = millis();
  nh.spinOnce ();

  if (curr_time - last_comm_time > LOST_COMM_INTERVAL)
  {
    for (int i = 0; i < INP_LEN; i++)
      recv_buff[i] = 0;
    last_comm_time = curr_time;
    digitalWrite (LED, LOW);
  }

  updatePinBuff();
  writeMotors();
  if (curr_time - last_write_time > WRITE_INTERVAL)
  {
    writeCanBus();
    last_write_time = curr_time;
  }
}



void callback (const std_msgs::Int32MultiArray& msg) {
  if (msg.layout.dim[0].size != INP_LEN ||
    msg.layout.dim[0].stride != INP_LEN || msg.layout.data_offset != 0)
  {
    return;
  }

  for (int i = 0; i < INP_LEN; i++)
    recv_buff[i] = msg.data[i];

  last_comm_time = curr_time;
  digitalWrite (LED, HIGH);
}

void updatePinBuff () {
  for (int i = 0; i < INP_LEN; i++)
  {
    int pwm, en, ina, inb;
    en = recv_buff[i] == 0;
    ina = recv_buff[i] < 0;
    inb = recv_buff[i] > 0;
    pwm = 0xFF - (inb ? recv_buff[i] : -recv_buff[i]);

    pin_buff[4*i+0] = pwm;
    pin_buff[4*i+1] = en;
    pin_buff[4*i+2] = ina;
    pin_buff[4*i+3] = inb;
  }
}

void writeCanBus () {
  CAN.sendMsgBuf(ID_A, STD_FRAME, 8, pin_buff+8);
  CAN.sendMsgBuf(ID_B, STD_FRAME, 8, pin_buff+16);
}

void writeMotors () {
  // motor_buff in this order: pwm, en, ina, inb
  pwmWrite (PWM1,     motor_buff[0]*PWMMAX/0xFF);
  digitalWrite (EN1,  motor_buff[1]);
  digitalWrite (INA1, motor_buff[2]);
  digitalWrite (INB1, motor_buff[3]);
  pwmWrite (PWM2,     motor_buff[4]*PWMMAX/0xFF);
  digitalWrite (EN2,  motor_buff[5]);
  digitalWrite (INA2, motor_buff[6]);
  digitalWrite (INB2, motor_buff[7]);
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
