
#include <SoftwareSerial.h>
#include<Wire.h>
#include "DynamixelMotor.h"
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#define rxPin 3
#define txPin 4
#define ledPin 13
SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);
const long unsigned int baudrate = 1000000; //baud rate of servo dont touch
HardwareDynamixelInterface interface(Serial);
int16_t speed=1023; //max speed
DynamixelMotor motor1(interface, 1); //motor1
DynamixelMotor motor2(interface, 2); //motor 2


unsigned char pm1 [150]; //0 - 180 degress is from 20-200;
unsigned char pm2 [150];
int len = 150;

void setup(){
 //Serial.begin(115200);
 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
 pm1[0] = 90;
 pm2[0] = 90;
  

 pinMode(rxPin, INPUT);
 pinMode(txPin, OUTPUT);
 xbee.begin(57600);
 pinMode(13,OUTPUT);
 digitalWrite(13,LOW);
 
 interface.begin(baudrate);
 motor1.enableTorque();  //start motor
 motor2.enableTorque();
 motor1.jointMode(204, 820); //dynamixel 0 -180
 motor2.jointMode(204, 820);
 motor1.speed(1023);
 motor2.speed(1023);
 
}

void sendAccel(){
  uint8_t payload [] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  payload[0] = 250;
  payload[1] = (uint8_t)10;
  payload[2] = (uint8_t)0;
  payload[3] = 0;
  payload[4] = 0;
  payload[5] = 0;
  payload[6] = 0;
  payload[7] = AcX >> 8 & 0xff;
  payload[8] = AcX & 0xff;
  payload[9] = AcY >> 8 & 0xff;
  payload[10] = AcY & 0xff;
  payload[11] = AcZ >> 8 & 0xff;
  payload[12] = AcZ & 0xff;
  payload[13] = 251;
  xbee.write(payload,14);
}

void runMotorsWithNoData(){
  unsigned long times = millis();
  double timestep = 700.0/((double)len); //calulates timestep for .7second drop depending on num of positions
  for(int i = 0; i < len- 1; i++){
    double goal1 = ((804.0 - 200)/(180.0))*((double)pm1[i]) + 200; //scales position val to correct motor val
    double goal2 = ((804.0 - 200)/(180.0))*((double)pm2[i]) + 200;
    motor1.goalPosition(goal1); //motor1 goto
    motor2.goalPosition(goal2);
    while(millis() - times < timestep){ //waits until next time step.
    }
    times = millis();
  }
}


void runMotorsWithLotsData(){
  uint8_t payload [] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  unsigned long times = millis();
  unsigned long tottimes = millis();
  double timestep = 700.0/((double)len); //calulates timestep for .7second drop depending on num of positions
  for(int i = 0; i < len- 1; i++){
    double goal1 = ((804.0 - 200)/(180.0))*((double)pm1[i]) + 200; //scales position val to correct motor val
    double goal2 = ((804.0 - 200)/(180.0))*((double)pm2[i]) + 200;
    motor1.goalPosition(goal1); //motor1 goto
    motor2.goalPosition(goal2);/*
    uint16_t m1 = (motor1.currentPosition()- (uint16_t)200);
    uint16_t temp1 = (uint16_t)(180.0*((float)m1)/604.0);
    uint16_t m2 = (motor2.currentPosition()- (uint16_t)200);
    uint16_t temp2 = (uint16_t)(180.0*((float)m2)/604.0);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    payload[0] = 250;
    payload[1] = (uint8_t)0;
    payload[2] = (uint8_t)i;
    payload[3] = temp1 >> 8 & 0xff;
    payload[4] = temp1 & 0xff;
    payload[5] = temp2 >> 8 & 0xff;
    payload[6] = temp2 & 0xff;
    payload[7] = GyX >> 8 & 0xff;
    payload[8] = GyX & 0xff;
    payload[9] = GyY >> 8 & 0xff;
    payload[10] = GyY & 0xff;
    payload[11] = GyZ >> 8 & 0xff;
    payload[12] = GyZ & 0xff;
    payload[13] = 251;
    xbee.write(payload,14);*/
    while(millis() - times < timestep){ //waits until next time step.
    }
    times = millis();
  } 
}


bool waitUntilDrop(){
  unsigned long times = millis();
  bool state = true;
  while(state){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    if(millis() - times > 10000){
      return false;
    }
    if(abs(AcX) < 5000 && abs(AcY) < 5000 && abs(AcZ)< 5000 ){
      delay(40);
      digitalWrite(13,HIGH);
      return true;
    }
  }
}


int readInCommand(){
  uint8_t data [14];
  unsigned long times = millis();
  if(xbee.available() > 13){
    xbee.readBytes(data,14);
    unsigned char start = data[0];
    unsigned char type = data[1];
    unsigned char sto = data[13];
    if(sto != 251){
      bool state = false;
      while(!state){
        if(xbee.read() == 251){
          state = true;
        }
      }
      return -1;
    }
    else if(type == 1){
      return 1;
    }
    else if(type == 2){
      return 2; 
    }
    else if(type == 3){
      return 3;
    }
    else if(type == 4){
      return 4;
    }
    else if(type == 10){
      return 10;
    }
    return 0;
  }
 return 0;
}


int readInMotorData(int motor,int index){
  uint8_t data [14];
  if(xbee.available() > 13){
    xbee.readBytes(data,14);
   // Serial.write(data,14);
    unsigned char start = data[0];
    unsigned char type = data[1];
    unsigned char sto = data[13];
    if(type == 10){
      return 10;
    }
    else if(type == 11){
      return 11;
    }
    else{
      len = data[2];
      if(motor == 1){
        for(int i = 0; i < 10; i++){
          pm1[i+index] = data[3+i]; 
        }
      }
      else{
        for(int i = 0; i < 10; i++){
          pm2[i+index] = data[3+i]; 
        }
      }
    }
    return 1;
  }
  return 0;
}


void loop(){
  motor1.goalPosition(((804.0 - 200)/(180.0))*((double)pm1[0]) + 200); //just go to beginning values at start when doing nothing
  motor2.goalPosition(((804.0 - 200)/(180.0))*((double)pm2[0]) + 200);
  motor1.led(false); //leds can be used to debug
  motor2.led(false);
  
  digitalWrite(13,LOW);
  int command = readInCommand();
 
  
  if(command ==1){
   digitalWrite(13,HIGH);
   //Serial.println("reading");
   int counter = 0;
   int index = 0;
   int motor = 1;
   bool finished = false;
   while(!finished){
      int answer = readInMotorData(motor,index);
      if(answer == 1){
        index = index+10; 
      }
      if(answer ==11){
        index = 0;
        motor = 2;
      }
      else if(answer == 10){
        finished = true;
      }
    }
  }
  else if(command == 2){
    digitalWrite(13,HIGH);
    sendAccel();
  }
  else if(command == 3){
    digitalWrite(13,LOW);
    if(waitUntilDrop()){
       runMotorsWithLotsData();
       uint8_t payload [] = {250,10,0,0,0,0,0,0,0,0,0,0,0,251};
       xbee.write(payload,14);
    }
    else{
      uint8_t payload [] = {250,10,0,0,0,0,0,0,0,0,0,0,0,251};
       xbee.write(payload,14);
    }
   }
  else if(command == 4){
    digitalWrite(13,LOW);
    if(waitUntilDrop()){
       runMotorsWithNoData();
       uint8_t payload [] = {250,10,0,0,0,0,0,0,0,0,0,0,0,251};
       xbee.write(payload,14);
    }
    else{
      uint8_t payload [] = {250,10,0,0,0,0,0,0,0,0,0,0,0,251};
       xbee.write(payload,14);
    }
   }
}
