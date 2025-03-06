#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// WiFi Setting
const char* SSID = "xxx";                    //  WiFi Name      
const char* PASSWORD = "xxxxxx";            //  WiFi Password

#define SCL 2  //IO_2 of ESP_01s connect the SCL of driver board
#define SDA 0  //IO_0 of ESP_01s connect the SDA of driver board
const int MPU = 0x68;

// UDP Setting
WiFiUDP Udp;
unsigned int localUdpPort = 6666;                 // choose a port addres
char incomingPacket[UDP_TX_PACKET_MAX_SIZE + 1];  // save the incoming UDP Packet
String ReplyBuffer = "got";

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default address 0x40

#define MOTOR_NUM 14   //Head12, LF 3 4 5, RF 6 7 8, LH 9 10 11, RH 12 13 14
#define PWM_Offset 0  // For Install, the start pin of motors, on board PCA9685
/*
 * DSM44 --> [0, 180]
 * Reference pulse duration: [0.5ms, 2.5ms] --> [0, 180]
 * 50Hz --> 20ms
 * SERVOMIN = 4096*0.5/20 = 102.4 --> 102
 * SERVOMAX = 4096*2.5/20 = 512
*/
int motorValues[MOTOR_NUM+1];
int curAngle[MOTOR_NUM];
bool goflag = false;

/**************************************Setup*****************************************/

void setup() {
  Serial.begin(115200);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Udp.begin(localUdpPort);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("UDP server on port %d\n", localUdpPort);

  Wire.begin(SDA, SCL);
  /*
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  */
  pwm.begin();
  pwm.setPWMFreq(50);  // the maximum PWM frequency 60Hz

  for(int i = 0; i < MOTOR_NUM+1; i++){
    motorValues[i] = 0;
  } 
}

/************************************************************************************/

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int n = Udp.read(incomingPacket, UDP_TX_PACKET_MAX_SIZE);
    incomingPacket[n] = 0;

    Serial.print("Received packet: ");
    Serial.println(incomingPacket);
    streamSplit(incomingPacket, ",");

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer.c_str());
    Udp.endPacket();

    goflag = true;

  } else {
    goflag = false;
  }

  if (goflag) {
    ctrlMotors();
  }
}

/***********************************************************************************/

void streamSplit(char* streamString, const char* flag) {
  int n = 0;
  char* result = NULL;

  result = strtok(streamString, flag);
  while (result != NULL) {
    String temp = String(result);
    motorValues[n++] = temp.toInt();
    result = strtok(NULL, flag);
  }

  for (int i = 0; i < MOTOR_NUM; i++) {
    curAngle[i] = motorValues[i];
    Serial.print(curAngle[i]);
    Serial.print(", ");
  }
  Serial.println("......");                                                                                                                                      
}

void ctrlMotors() {
  // *** left 0+, right 180-
  int pulse0 = map(curAngle[0]+90, 0, 180, 118, 510);       //Head rotation  turn left 90+, turn right 90-
  pwm.setPWM(0, 0, pulse0);
  
  int pulse1 = map(curAngle[1]+90, 0, 180, 100, 514);        //Head tilt up and down
  pwm.setPWM(1, 0, pulse1);

  int pulse2 = map(curAngle[2]+45, 0, 180, 105, 500);       //left fore leg servo1
  pwm.setPWM(2, 0, pulse2);
  int pulse3 = map(curAngle[3]-45, 0, 180, 130, 533);       //left fore leg servo2
  pwm.setPWM(3, 0, pulse3);
  int pulse4 = map(90-curAngle[4], 0, 180, 124, 524);       //left fore leg servo3
  pwm.setPWM(4, 0, pulse4);

  int pulse5 = map(135-curAngle[5], 0, 180, 128, 525);        //right fore leg servo1
  pwm.setPWM(5, 0, pulse5);
  int pulse6 = map(225-curAngle[6], 0, 180, 118, 520);        //right fore leg servo2
  pwm.setPWM(6, 0, pulse6);
  int pulse7 = map(curAngle[7]+90, 0, 180, 124, 532);       //right fore leg servo3
  pwm.setPWM(7, 0, pulse7);

  int pulse8 = map(curAngle[8]+45, 0, 180, 128, 532);        //left hind leg servo1
  pwm.setPWM(8, 0, pulse8);
  int pulse9 = map(curAngle[9]-45, 0, 180, 130, 536);        //left hind leg servo2
  pwm.setPWM(9, 0, pulse9);
  int pulse10 = map(90-curAngle[10], 0, 180, 118, 520);       //left hind leg servo3
  pwm.setPWM(10, 0, pulse10);

  int pulse11 = map(135-curAngle[11], 0, 180, 118, 510);        //right hind leg servo1
  pwm.setPWM(11, 0, pulse11);
  int pulse12 = map(225-curAngle[12], 0, 180, 111, 518);       //right hind leg servo2
  pwm.setPWM(12, 0, pulse12);
  int pulse13 = map(curAngle[13]+90, 0, 180, 107, 492);       //right hind leg servo3
  pwm.setPWM(13, 0, pulse13);

  //Serial.println("move");
}
