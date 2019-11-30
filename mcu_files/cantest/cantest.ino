HardwareTimer timer(3);

#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS_PIN PA4
MCP_CAN CAN (SPI_CS_PIN);

//-------------------------------------
// Pin Definitions

#define LED PC13

#define PWM1 PA9
#define INB1 PB8
#define INA1 PB7
#define EN1  PB0

#define PWM2 PA8
#define INB2 PB6
#define INA2 PA3
#define EN2  PA2

// Function Declarations
void initAndClearOutputs ();
void initPWM ();
void initCAN ();

#define SEND_LED EN2
#define RECV_LED PWM1

//-------------------------------------
// Main

#define INTERVAL 2000
unsigned long timeLast;
unsigned char recvBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char sendBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

#define SEND_ID 0x00
#define STD_FRAME 0

void setup () {
  initAndClearOutputs ();
  initCAN ();
  timeLast = millis();
}

void loop () {
  unsigned long timeCurr = millis();

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    unsigned char len = 0;
    CAN.readMsgBuf(&len, recvBuf);
    unsigned long canID = CAN.getCanId();

    if (len == 0)
      digitalWrite (LED, LOW);
    else {
      digitalWrite (LED, HIGH);
      digitalWrite (RECV_LED, recvBuf[0]);
    }
  }

  if (timeCurr - timeLast >= INTERVAL)
  {
    sendBuf[0] = !sendBuf[0];
    digitalWrite (SEND_LED, sendBuf[0]);
    CAN.sendMsgBuf (SEND_ID, STD_FRAME, 1, sendBuf);
    timeLast = timeCurr;
  }
}


//-------------------------------------
// Function definitions

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

void initPWM () {
  timer.setPrescaleFactor(4);
  timer.setOverflow(1024);
  timer.refresh();
}

void initAndClearOutputs () {
  pinMode (LED, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (INB1, OUTPUT);
  pinMode (INA1, OUTPUT);
  pinMode (EN1, OUTPUT);
  pinMode (PWM2, OUTPUT);
  pinMode (INB2, OUTPUT);
  pinMode (INA2, OUTPUT);
  pinMode (EN2, OUTPUT);

  digitalWrite (LED, HIGH);
  digitalWrite (PWM1, HIGH);
  digitalWrite (INB1, HIGH);
  digitalWrite (INA1, HIGH);
  digitalWrite (EN1, HIGH);
  digitalWrite (PWM2, HIGH);
  digitalWrite (INB2, HIGH);
  digitalWrite (INA2, HIGH);
  digitalWrite (EN2, HIGH);
}
