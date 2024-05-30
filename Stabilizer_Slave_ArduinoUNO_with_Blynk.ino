/*  This MCE242 project was created by 65070502202 Kantapong Premyodin and 65070502204 Chutipon Likitparinya
    The project name is 3-Axis stabilizer with adjust value and PID
    NOTE : First we use esp32 to drive motor but esp32 cannot hold that much voltage so we have to use another board 
    and that board is Arduino UNO R3 by I2C communication */
    
/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]     DEFINE FOR I2C COMMUNICATION     [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
#include <Wire.h>
#define slaveAddress 0x08  //you have to assign an 8-bit address to Slave
int8_t dataArray[3] = { 0 , 0 , 0 };
long raw_ROW, raw_PITCH, raw_YAW;

/*[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][]       DEFINE FOR MOTOR CONTROL       [][][][][][][][][][][][][][][]
[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]*/
long map_ROW, map_PITCH, map_YAW;
int state = 0;

int RmotorPin1 = 2;
int RmotorPin2 = 3;
int RmotorPin3 = 4;
int RmotorPin4 = 5;
int PmotorPin1 = 6;
int PmotorPin2 = 7;
int PmotorPin3 = 8;
int PmotorPin4 = 9;
int YmotorPin1 = 10;
int YmotorPin2 = 11;
int YmotorPin3 = 12;
int YmotorPin4 = 13;

void setup() {
  Wire.begin(slaveAddress);
  Serial.begin(115200);
  Wire.onReceive(receiveEvent);  //You need to declre it in setup() to receive data from Master

  pinMode(RmotorPin1, OUTPUT);
  pinMode(RmotorPin2, OUTPUT);
  pinMode(RmotorPin3, OUTPUT);
  pinMode(RmotorPin4, OUTPUT);
  pinMode(PmotorPin1, OUTPUT);
  pinMode(PmotorPin2, OUTPUT);
  pinMode(PmotorPin3, OUTPUT);
  pinMode(PmotorPin4, OUTPUT);
  pinMode(YmotorPin1, OUTPUT);
  pinMode(YmotorPin2, OUTPUT);
  pinMode(YmotorPin3, OUTPUT);
  pinMode(YmotorPin4, OUTPUT);
}

void loop() {
  raw_ROW = dataArray[0];
  raw_PITCH = dataArray[1];
  raw_YAW = dataArray[2];

  Serial.print(" || ROW = ");
  Serial.print(dataArray[0], DEC);
  Serial.print(" || PITCH = ");
  Serial.print(dataArray[1], DEC);
  Serial.print(" || YAW = ");
  Serial.print(dataArray[2], DEC);

  if (raw_ROW > 14) {
    map_ROW = map(raw_ROW, 0, 127, 4, 2);
  }
  if (raw_ROW < -14) {
    map_ROW = map(raw_ROW, -128, 0, -2, -4);
  }
  if (raw_ROW <= 14 && raw_ROW >= -14) {
    map_ROW = 0;
  }

  if (raw_PITCH > 14) {
    map_PITCH = map(raw_PITCH, 0, 127, 4, 2);
  }
  if (raw_PITCH < -14) {
    map_PITCH = map(raw_PITCH, -128, 0, -2, -4);
  }
  if (raw_PITCH <= 14 && raw_PITCH >= -14) {
    map_PITCH = 0;
  }

  if (raw_YAW > 14) {
    map_YAW = map(raw_YAW, 0, 127, 4, 2);
  }
  if (raw_YAW < -14) {
    map_YAW = map(raw_YAW, -128, 0, -2, -4);
  }
  if (raw_YAW <= 14 && raw_YAW >= -14) {
    map_YAW = 0;
  }

  Serial.print(" ---- mapROW = ");
  Serial.print(map_ROW);
  Serial.print(" || mapPITCH = ");
  Serial.print(map_PITCH);
  Serial.print(" || mapYAW = ");
  Serial.print(map_YAW);

  //For ROW axis
  if (state == 0) {
    //+ ROW axis
    if (map_ROW > 0) {
      Serial.println(" ||| +ROW");
      digitalWrite(RmotorPin1, HIGH);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, HIGH);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, HIGH);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, HIGH);
      delay(abs(map_ROW));
    }
    //- ROW axis
    if (map_ROW < 0) {
      Serial.println(" ||| -ROW");
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, HIGH);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, HIGH);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, LOW);
      digitalWrite(RmotorPin2, HIGH);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
      digitalWrite(RmotorPin1, HIGH);
      digitalWrite(RmotorPin2, LOW);
      digitalWrite(RmotorPin3, LOW);
      digitalWrite(RmotorPin4, LOW);
      delay(abs(map_ROW));
    }
    if (map_ROW == 0) {
      state = 1;
    }
  }
//state = 1
  //For PITCH axis
  if (state == 1) {
    //+ PITCH axis
    if (map_PITCH > 0) {
      Serial.println(" ||| +PITCH");
      digitalWrite(PmotorPin1, HIGH);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, HIGH);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, HIGH);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, HIGH);
      delay(abs(map_PITCH));
    }
    //- PITCH axis
    if (map_PITCH < 0) {
      Serial.println(" ||| -PITCH");
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, HIGH);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, HIGH);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, LOW);
      digitalWrite(PmotorPin2, HIGH);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
      digitalWrite(PmotorPin1, HIGH);
      digitalWrite(PmotorPin2, LOW);
      digitalWrite(PmotorPin3, LOW);
      digitalWrite(PmotorPin4, LOW);
      delay(abs(map_PITCH));
    }
    if (map_PITCH == 0) {
      state = 2;
    }
  }
//state = 2
  //For YAW axis
  if (state == 2) {
    //+ YAW axis
    if (map_YAW > 0) {
      Serial.println(" ||| +YAW");
      digitalWrite(YmotorPin1, HIGH);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, HIGH);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, HIGH);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, HIGH);
      delay(abs(map_YAW));
    }
    //- PITCH axis
    if (map_YAW < 0) {
      Serial.println(" ||| -YAW");
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, HIGH);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, HIGH);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, LOW);
      digitalWrite(YmotorPin2, HIGH);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
      digitalWrite(YmotorPin1, HIGH);
      digitalWrite(YmotorPin2, LOW);
      digitalWrite(YmotorPin3, LOW);
      digitalWrite(YmotorPin4, LOW);
      delay(abs(map_YAW));
    }
    if (map_YAW == 0) {
      state = 0;
    }
  }
//state = 3
  if (map_ROW != 0 && state != 0) {
    state = 0;
  }
  if (map_PITCH != 0 && state == 2) {
    state = 1;
  }
//Stop State
  if (map_ROW == 0 && map_PITCH == 0 && map_YAW == 0){
    Serial.println(" ||| STOP");
  }
  Serial.print(" \\ state = ");
  Serial.print(state);
}

void receiveEvent(int howmany)  //howmany = Wire.write()executed by Master ESP
{
  while (1 < Wire.available()) {
    for (int i = 0; i < howmany; i++) {
      dataArray[i] = Wire.read();
    }
  }
}
