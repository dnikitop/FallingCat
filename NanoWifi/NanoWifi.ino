#include <SoftwareSerial.h>
#include "DynamixelMotor.h"
#define rxPin 3
#define txPin 2
#define ledPin 13
SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);
const long unsigned int baudrate = 1000000; //baud rate of servo dont touch
HardwareDynamixelInterface interface(Serial);
int16_t speed=1023; //max speed
DynamixelMotor motor(interface, 1); //motor1
DynamixelMotor motor2(interface, 2); //motor 2


unsigned char pm1 [300]; //0 - 180 degress is from 20-200;
unsigned char pm2 [300];
int len;
unsigned char rm1 [300];
unsigned char rm2 [300];

void setup(){
 pm1[0] = 90;
 pm2[0] = 90;
 interface.begin(baudrate);
 pinMode(rxPin, INPUT);
 pinMode(txPin, OUTPUT);
 xbee.begin(9600);
 
 motor.enableTorque();  //start motor
 motor2.enableTorque();
 motor.jointMode(204, 820); //dynamixel 0 -180
 motor2.jointMode(204, 820);
 motor.speed(1023);
 motor2.speed(1023);
}

void readInData(){ //read in transmitted motor position
  int counter = 0;
  bool start = false;
  bool start1 = false;
  bool start2 = false;
  while(!start){ //get the length first
    if(xbee.available()){
      unsigned char input = (unsigned char)xbee.read();
      len = input;
      start = true;
    }
  }
  counter = 0;
  while(!start1){ //get the position for motor1
    if(xbee.available()){
      unsigned char input = (unsigned char)xbee.read();
      if(input == 182){ //stop until motor1stop command
        start1=true;
      }
      else{
        pm1[counter] = input;
        counter++;
      }
    }
  }
  counter = 0;
  while(!start2){ //get the position for motor2
    if(xbee.available()){
      unsigned char input = (unsigned char)xbee.read();
      if(input == 183){
        start2=true;
      }
      else{
        pm2[counter] = input;
        counter++;
      }
    }
  }
}

void runMotors(){
  unsigned long times = millis();
  double timestep = 700.0/((double)len); //calulates timestep for .7second drop depending on num of positions
  for(int i = 0; i < len-1; i++){
    double goal1 = ((804.0 - 200)/(180.0))*((double)pm1[i]) + 200; //scales position val to correct motor val
    double goal2 = ((804.0 - 200)/(180.0))*((double)pm2[i]) + 200;
    motor.goalPosition(goal1); //motor1 goto
    motor2.goalPosition(goal2);

    /* Not Implemented Fully
    double m1 = ((180.0)/(804.0-200))*(motor1.currentPosition()-200.0);
    double m2 = ((180.0)/(804.0-200))*(motor1.currentPosition()-200.0);
    rm1[i] = (unsigned char)m1;
    rm2[i] = (unsigned char)m2;
  */
    
    //here would go gyro and positon data storage
    while(millis() - times < timestep){ //waits until next time step.
    }
    times = millis();
  }
}
void loop(){
  
  motor.goalPosition(((804.0 - 200)/(180.0))*((double)pm1[0]) + 200); //just go to beginning values at start when doing nothing
  motor2.goalPosition(((804.0 - 200)/(180.0))*((double)pm2[0]) + 200);
  motor.led(false); //leds can be used to debug
  motor2.led(false);
  
  if(xbee.available()){
    unsigned char input = (unsigned char)xbee.read(); //look for wireless command
    if(input== 181){ //181 is motor position transmit command
      motor2.led(true);
      readInData();
    }
    else if(input == 184){ //184 is move command
      motor.led(true);
      runMotors();
      delay(1000);
    }
    else if(input == 185){ //185 and send data back command (not implemented yet)
      motor.led(true);
      runMotors();
      delay(1000);
      for(int i = 0; i < len-1; i++){
        xbee.write(rm1[i]);
      }
      for(int i = 0; i < len-1; i++){
        xbee.write(rm2[i]);
      }
      //send feedback
    }
    else{
    }
  }
}
