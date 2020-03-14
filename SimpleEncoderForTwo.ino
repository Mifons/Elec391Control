
/*
 * Display number on seven segment display with Arduino
 * Written by Ahmad Shamshiri for Robojax.com
Date modified: Jun 11, 2018 at 17:20 at Ajax, Ontario, Canada
watch video instruction for this code:https://youtu.be/-g6Q9lSHDzg

Code is provided for free AS IS without warranty. 
You can share this acode as long as you keep the above note intact.
 */



/*--------------Variables for Moving the Motor using L9110S--------------*/
#include "QuadEncoder.h"

//LeftMotor
const int A1A = 5;
const int A1B = 6;

//RightMotor
const int B1A = 7;
const int B1B = 8;

int motorSpeedLeft, motorSpeedRight;
const int Speed = 180;
/*-----------------------------------------------------------*/

QuadEncoder motorLeft(1, 0, 1, 1);
QuadEncoder motorRight(2, 2, 3, 1);


void setup() {
  //Motor Setup
  pinMode(A1A,OUTPUT);// define pin as output for Left Motor
  pinMode(A1B,OUTPUT);
  pinMode(B1A,OUTPUT);// define pin as output for Right Motor
  pinMode(B1B,OUTPUT);

  Serial.begin(9600);
  motorSpeedLeft = Speed; //Set initialspeed to zero
  motorSpeedRight = Speed;

  //Encoder Setup
  motorLeft.setInitConfig();
  motorLeft.init();
  motorRight.setInitConfig();
  motorRight.init();
  
 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    // send an intro:
  Serial.println("Move me");
  
}

long positionLeft  = -999;
long positionRight  = -999;
  
void loop() {
  long newLeft;
  long newRight;
  if (Serial.available() > 0) {
      char thisChar = Serial.read();
     
      moveMotorLeft(thisChar);
      moveMotorRight(thisChar);
      
  }

  newLeft = motorLeft.read();
  newRight = motorRight.read();
  
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(" ");
    Serial.print("Right = ");
    Serial.println(newRight);
    
    positionLeft = newLeft;
    positionRight = newRight;
  }
 
}




 //Motor L9110S function for Left Motor
void moveMotorLeft(char d)
{
    if(d =='F'){
      analogWrite(A1A,LOW);
      analogWrite(A1B,motorSpeedLeft); 
    }else if(d =='B'){
      analogWrite(A1A,motorSpeedLeft);
      analogWrite(A1B,LOW);    
    }else{
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF      
      analogWrite(A1A,LOW);
      analogWrite(A1B,LOW);     
    }
}// move end 

//For right motor
void moveMotorRight(char d)
{
    if(d =='F'){
      analogWrite(B1A,LOW);
      analogWrite(B1B,motorSpeedRight); 
    }else if(d =='B'){
      analogWrite(B1A,motorSpeedRight);
      analogWrite(B1B,LOW);    
    }else{
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF      
      analogWrite(B1A,LOW);
      analogWrite(B1B,LOW);     
    }
}// move end 
