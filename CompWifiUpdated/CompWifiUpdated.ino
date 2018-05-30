#include <Printers.h>
#include <XBee.h>


#include <SoftwareSerial.h>

#define rxPin 3
#define txPin 2
#define ledPin 13
SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);

unsigned char pos1 [150];
unsigned char pos2 [150];
unsigned char len;

void setup(){
 pinMode(rxPin, INPUT);
 pinMode(txPin, OUTPUT);
 xbee.begin(57600);
 Serial.begin(115200);
 pinMode(13,OUTPUT);
}


int readInData(){
  unsigned long times = millis();
  uint8_t data[14];
  if(xbee.available() > 13){
    xbee.readBytes(data,14);
    unsigned char start = data[0];
    unsigned char type = data[1];
    unsigned char times = data[2];
    uint16_t pos1 = ((uint16_t)data[3] << 8) + data[4];
    uint16_t pos2 = ((uint16_t)data[5] << 8) + data[6];
    uint16_t gyx = ((uint16_t)data[7] << 8) + data[8];
    uint16_t gyy = ((uint16_t)data[9] << 8) + data[10];
    uint16_t gyz = ((uint16_t)data[11] << 8) + data[12];
    unsigned char sto = data[13];/*
    Serial.print(type);
    Serial.print(" ");
    Serial.print(times);
    Serial.print(" ");
    Serial.print(pos1);
    Serial.print(" ");
    Serial.print(pos2);
    Serial.print(" ");
    Serial.print(gyx);
    Serial.print(" ");
    Serial.print(gyy);
    Serial.print(" ");
    Serial.print(gyz);
    */
    Serial.write(data,14);
    if(sto != 251){
      bool state = false;
      while(!state){
        if(xbee.read() == 251){
          state = true;
        }
      }
      return -1;
    }
    else if(type == 10){
      return 0;
    }
    return 1;
  }
  return 1;
}


void loop(){
  unsigned char option = 0;
  if(Serial.available()){
    option = (unsigned char)Serial.read();
  }
  if(option == 1){ //send
    digitalWrite(13,HIGH);
    bool finished = false;
    uint8_t sendCommand[] = {250,1,0,0,0,0,0,0,0,0,0,0,0,251};
    xbee.write(sendCommand,14);
    delay(100);
    uint8_t temp [14];
    int counter = 0;
    while(!finished){   
      if(Serial.available() > 13){
        Serial.readBytes(temp,14);
        xbee.write(temp,14);
        counter++;
        if(temp[1] == 10){
        finished = true;
        }
      }
 
    }
      digitalWrite(13,LOW);
  }
  else if(option ==2){ //get accell
   digitalWrite(13,HIGH);
   xbee.flush();
   uint8_t sendCommand[] = {250,2,0,0,0,0,0,0,0,0,0,0,0,251};
   xbee.write(sendCommand,14);
   bool finished = false;
   while(!finished){
    uint8_t temp [14];
    if(xbee.available() > 13){
      xbee.readBytes(temp,14);
      Serial.write(temp,14);
      finished = true;
      digitalWrite(13,LOW);
    }
   }
   xbee.flush();
  }
  else if(option == 3){
    uint8_t sendCommand[] = {250,3,0,0,0,0,0,0,0,0,0,0,0,251};
    xbee.write(sendCommand,14);
    bool finished = false;
    while(!finished){
      if(readInData() == 0){
        finished = true;
      }
    }
  }
  else if(option == 4){
    uint8_t sendCommand[] = {250,4,0,0,0,0,0,0,0,0,0,0,0,251};
    xbee.write(sendCommand,14);
    bool finished = false;
    while(!finished){
      if(readInData() == 0){
        finished = true;
      }
    }
  }
  xbee.flush();
}


