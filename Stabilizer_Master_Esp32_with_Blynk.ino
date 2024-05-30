/*  This MCE242 project was created by 65070502202 Kantapong Premyodin and 65070502204 Chutipon Likitparinya
    The project name is 3-Axis stabilizer with adjust value and PID
    NOTE : First we use esp32 to drive motor but esp32 cannot hold that much voltage so we have to use another board 
    and that board is Arduino UNO R3 by I2C communication */

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]      INCLUDE LIBRARY FOR MPU6050     [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][]   INCLUDE LIBRARY FOR I2C COMMUNICATION  [][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
#include <Wire.h>

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][]              INCLUDE LIBRARY AND DEFINE BLYNK INFO               [][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
#define BLYNK_TEMPLATE_ID "TMPL6zOYvoFza"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "s2cs7kfO603Rcx9-3eNtWDw3zi-LvkDw"
#define BLYNK_PRINT Serial
#include <SPI.h>
#include <Ethernet.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials
// Set password to "" for open networks
char ssid[] = "GaplnwZaZa";
char pass[] = "gaplnwzaza";
/*    ^^^
       |
       |
       Set to 2.4 Ghz wifi by your | wifi name - ssid | wifi password - pass
*/

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]     DEFINE FOR I2C COMMUNICATION     [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
#define slaveAddress 0x08  //you have to assign an 8-bit address to Slave Arduino UNO R3
byte dataArray[3] = { 0 , 0 , 0 };  //To transmit data to Slave using I2C the data type must be byte data for stable transmission

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]     DEFINE FOR MPU - QUATERNION      [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3] = { 0 , 0 , 0 };
float ROW, PITCH, YAW;

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]     DEFINE FOR MOTOR CONTROL PIN     [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
// #define MOTORA_EN 2  // PWM pin for controlling motor ROW axis speed 
// #define MOTORA_IN1 4  // Direction control pin +
// #define MOTORA_IN2 2  // Direction control pin -
// #define MOTORB_EN 2   // PWM pin for controlling motor PITCH axis speed
// #define MOTORB_IN1 0   // Direction control pin +
// #define MOTORB_IN2 4   // Direction control pin -
// #define MOTORC_EN 2   // PWM pin for controlling motor YAW axis speed
// #define MOTORC_IN1 0   // Direction control pin +
// #define MOTORC_IN2 2   // Direction control pin -

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]        DEFINE FOR PID CONTROL        [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
unsigned long Present_time, Previous_time;
float Delta_time;
int Present_state, Next_state;

float Present_error_ROW, Intergral_error_ROW, Devirative_error_ROW, Previous_error_ROW;
float Present_error_PITCH, Intergral_error_PITCH, Devirative_error_PITCH, Previous_error_PITCH;
float Present_error_YAW, Intergral_error_YAW, Devirative_error_YAW, Previous_error_YAW;

float out_ROW;
float out_PITCH;
float out_YAW;

float set_point_ROW = 0;
float set_point_PITCH = 0;
float set_point_YAW = 0;
/*    ^^^
       |
       |
       For adjust the position when you don't want to connected with blynk slider object
*/

float Min_out = -128;
float Max_out = 127;
float kp = 5;//10
float ki = 0.0010;//0.008
float kd = 39;//75
/*    ^^^
       |
       |
       For Optimize PID CONTROL
*/

void setup() {
//Define for MPU and I2C communication
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.begin(115200);

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

//Connected with blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

//Define OUTPUT for 3-axis motor pin
  // pinMode(MOTORA_EN, OUTPUT);
  // pinMode(MOTORA_IN1, OUTPUT);
  // pinMode(MOTORA_IN2, OUTPUT);
  // pinMode(MOTORB_EN, OUTPUT);
  // pinMode(MOTORB_IN1, OUTPUT);
  // pinMode(MOTORB_IN2, OUTPUT);
  // pinMode(MOTORC_EN, OUTPUT);
  // pinMode(MOTORC_IN1, OUTPUT);
  // pinMode(MOTORC_IN2, OUTPUT);
}

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][]    RECIEVE VALUE FROM BLYNK SLIDER     [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
BLYNK_WRITE ( V1 ) {
  set_point_ROW = param.asInt();
}

BLYNK_WRITE ( V2 ) {
  set_point_PITCH = param.asInt();
}

BLYNK_WRITE ( V3 ) {
  set_point_YAW = param.asInt();
}

void loop() {
  Blynk.run();
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("Row = ");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print(" | Pitch = ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(" | Yaw = ");
    Serial.print(ypr[0] * 180 / M_PI);

    ROW = ypr[2] * 180 / M_PI;
    PITCH = ypr[1] * 180 / M_PI;
    YAW = ypr[0] * 180 / M_PI;

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]                PID                   [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
  Present_time = micros();
  Delta_time = (float)(Present_time  - Previous_time);

//ROW PID
  Present_error_ROW   = set_point_ROW - ROW ;
  Intergral_error_ROW  += Present_error_ROW * Delta_time;
  Devirative_error_ROW  = (Present_error_ROW - Previous_error_ROW)/Delta_time ;
  if (Intergral_error_ROW >= Max_out ) Intergral_error_ROW = Max_out;
  else if(Intergral_error_ROW <= Min_out ) Intergral_error_ROW = Min_out;
  out_ROW = (kp*Present_error_ROW) + (ki*Intergral_error_ROW) + (kd*Devirative_error_ROW) ;
  if (out_ROW >= Max_out ) out_ROW = Max_out;
  else if (out_ROW <= Min_out ) out_ROW = Min_out;
  Previous_error_ROW = Present_error_ROW;

//PITCH PID
  Present_error_PITCH   = set_point_PITCH - PITCH ;
  Intergral_error_PITCH  += Present_error_PITCH * Delta_time;
  Devirative_error_PITCH  = (Present_error_PITCH - Previous_error_PITCH)/Delta_time ;
  if (Intergral_error_PITCH >= Max_out ) Intergral_error_PITCH = Max_out;
  else if(Intergral_error_PITCH <= Min_out ) Intergral_error_PITCH = Min_out;
  out_PITCH = (kp*Present_error_PITCH) + (ki*Intergral_error_PITCH) + (kd*Devirative_error_PITCH) ;
  if (out_PITCH >= Max_out ) out_PITCH = Max_out;
  else if (out_PITCH <= Min_out ) out_PITCH = Min_out;
  Previous_error_PITCH = Present_error_PITCH;
  
//YAW PID
  Present_error_YAW   = set_point_YAW - YAW ;
  Intergral_error_YAW  += Present_error_YAW * Delta_time;
  Devirative_error_YAW  = (Present_error_YAW - Previous_error_YAW)/Delta_time ;
  if (Intergral_error_YAW >= Max_out ) Intergral_error_YAW = Max_out;
  else if(Intergral_error_YAW <= Min_out ) Intergral_error_YAW = Min_out;
  out_YAW = (kp*Present_error_YAW) + (ki*Intergral_error_YAW) + (kd*Devirative_error_YAW) ;
  if (out_YAW >= Max_out ) out_YAW = Max_out;
  else if (out_YAW <= Min_out ) out_YAW = Min_out;
  Previous_error_YAW = Present_error_YAW;

  Previous_time  = Present_time;

//OUTPUT = out Print to check if it is in normal condition or not
  Serial.print(" ||| OUT_Row = ");
  Serial.print(out_ROW);
  Serial.print(" ||| OUT_Pitch = ");
  Serial.print(out_PITCH);
  Serial.print(" ||| OUT_Yaw = ");
  Serial.println(out_YAW);

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]          Send data to Slave          [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/

  dataArray[0] = out_ROW  ;
  dataArray[1] = out_PITCH  ;
  dataArray[2] = out_YAW  ;
  Wire.beginTransmission(slaveAddress); //Address is queued for checking if the slave is present
 
    Wire.write(dataArray,3);  //Data bytes are queued in local buffer
  
  Wire.endTransmission(); //All the above queued bytes are sent to slave on ACK handshaking

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]            MOTOR CONTROL             [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
//FOR ROW AXIS
/*
    if (out_ROW > 0 ) {
      // Move the motor forward
      digitalWrite(MOTORA_IN1, HIGH);
      digitalWrite(MOTORA_IN2, LOW);
      analogWrite(MOTORA_EN, abs(out_ROW)); 
    }
    if (out_ROW < 0 ) {
      // Stop the motor
      digitalWrite(MOTORA_IN1, LOW);
      digitalWrite(MOTORA_IN2, HIGH);
      analogWrite(MOTORA_EN, abs(out_ROW)); 
    }
    if (out_ROW = 0 ) {
      // Stop the motor
      digitalWrite(MOTORA_IN1, LOW);
      digitalWrite(MOTORA_IN2, LOW);
      analogWrite(MOTORA_EN, 0);
    }

//FOR PITCH AXIS
    if (out_PITCH > 0 ) {
      // Move the motor forward
      digitalWrite(MOTORB_IN1, HIGH);
      digitalWrite(MOTORB_IN2, LOW);
      analogWrite(MOTORB_EN, abs(out_PITCH));
    }
    if (out_PITCH < 0 ) {
      // Stop the motor
      digitalWrite(MOTORB_IN1, LOW);
      digitalWrite(MOTORB_IN2, HIGH);
      analogWrite(MOTORB_EN, abs(out_PITCH)); 
    }
    if (out_PITCH = 0 ) {
      // Stop the motor
      digitalWrite(MOTORB_IN1, LOW);
      digitalWrite(MOTORB_IN2, LOW);
      analogWrite(MOTORB_EN, 0); 
    }

//FOR YAW AXIS
    if (out_YAW > 0 ) {
      // Move the motor forward
      digitalWrite(MOTORC_IN1, HIGH);
      digitalWrite(MOTORC_IN2, LOW);
      analogWrite(MOTORC_EN, abs(out_YAW));
    }
    if (out_YAW < 0 ) {
      // Stop the motor
      digitalWrite(MOTORC_IN1, LOW);
      digitalWrite(MOTORC_IN2, HIGH);
      analogWrite(MOTORC_EN, abs(out_YAW)); 
    }
    if (out_YAW = 0 ) {
      // Stop the motor
      digitalWrite(MOTORC_IN1, LOW);
      digitalWrite(MOTORC_IN2, LOW);
      analogWrite(MOTORC_EN, 0); 
    }
*/
  }
}