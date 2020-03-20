#include "QuadEncoder.h"
/*--------------Variables for Moving the Motor using L9110S--------------*/

//LeftMotor
const int A1A = 5;
const int A1B = 6;

//RightMotor
const int B1A = 7;
const int B1B = 8;

int motorSpeedLeft, motorSpeedRight;
/*-----------------------------------------------------------*/


/*--------------Encoder Pulse CounterPID--------------*/
// Change these pin numbers to the pins connected to your encoder.
// Allowable encoder pins:
// 0, 1, 2, 3, 4, 5, 7, 30, 31 and 33
// Encoder on channel 1 of 4 available
// Phase A (pin0), PhaseB(pin1), 
QuadEncoder motorLeft(1, 0, 1, 1); //Left motor
QuadEncoder motorRight(2, 2, 3, 1);//Right motor
long rotationLeft, rotationRight; //This is variable for mapping degrees into number of pulses
/*-----------------------------------------------------------*/


/*--------------Variables for Computing PID--------------*/
   unsigned long lastTime;
  double InputLeft, OutputLeft, SetpointLeft, InputRight, OutputRight, SetpointRight;//input is in Pulse Count, output is motor speed, setpoint is in degrees
  double lastErrorLeft, lastErrorRight; //errorSum sums all the error
  double errorLeft, derivativeLeft, errorRight, derivativeRight;
  const double kp_L = 0.6, ki_L = 0.05, kd_L = 0.4;          //proportional, integral, derivative gains
  const double kp_R = 2.9, ki_R = 1, kd_R = 1.3;   
  int SampleTime = 20; //Default Sample Rate
  
   //Clamping Output
  #define MaxClamp  200
  #define MinClamp -200
  double ActuatorCommandLeft, ActuatorCommandRight; // Clamps output (Sets limit to output) initial value is 0
  
  //Clamping Integral
  bool IntegralClampLeft = false; //initial value is 0 so it doesn't clamp integral (turn off) in the beginning
  bool IntegralClampRight = false;
  double integralLeft, integralRight;

  //Low Pass Filter
  double CutoffFreqLeft, CutoffFreqRight; //rad/s
  //double LowPassFilterOut;
  double filtered_derivative_Left, filtered_derivative_Right;

  byte  User_Setpoint_Left, User_Setpoint_Right; //User's selected setpoint
/*-----------------------------------------------------------*/

void Compute_PID(){

  unsigned long nowTime = millis();//Obtain time first when computing PID
  //Calculate Change in time
  double delta_time = double(nowTime - lastTime);

    errorLeft = SetpointLeft - InputLeft;
    errorRight = SetpointRight - InputRight;
  
  
    //Integral Clamping for Left Motor 
    if(IntegralClampLeft){ //Switch on or off integral clamping for left motor
      integralLeft += errorLeft; //Integral computation
    }
    else{
      integralLeft = 0;
    }

    //Integral Clamping for Right Motor 
    if(IntegralClampRight){ //Switch on or off integral clamping for righ motor
      integralRight += errorRight; //Integral computation
    }
    else{
      integralRight = 0;
    }

    
    derivativeLeft = (errorLeft-lastErrorLeft); //Derivative computation
    derivativeRight = (errorLeft-lastErrorRight); //Derivative computation

    //Low pass Filter
    filtered_derivative_Left = derivativeLeft*CutoffFreqLeft/(derivativeLeft+CutoffFreqLeft);
    filtered_derivative_Right = derivativeRight*CutoffFreqRight/(derivativeRight + CutoffFreqRight);
    
    //Output
    OutputLeft = (kp_L * errorLeft + ki_L * integralLeft + kd_L * filtered_derivative_Left);
    OutputRight = (kp_R * errorRight + ki_R * integralRight + kd_R * filtered_derivative_Right);

    //Clamp Output
    ActuatorCommandLeft = ClampOutput(MinClamp, MaxClamp, OutputLeft);
    ActuatorCommandRight = ClampOutput(MinClamp, MaxClamp, OutputRight);

    //Clamp Integral
    IntegralClampLeft = ClampIntegral(ActuatorCommandLeft, OutputLeft);
    IntegralClampRight =  ClampIntegral(ActuatorCommandRight, OutputRight);
  
  //Store variables for next computation of PID
  lastErrorLeft = errorLeft;
  lastErrorRight = errorRight;

   lastTime = nowTime;
  
}



//This is for clamping the output signal (setting limits to output signal) before going into the actuator(in this case the motor controller)
double ClampOutput(double Min, double Max, double Output){
  if (Output > Max){
    return Max;
  }
  else if(Output < Min){
    return Min;
  }
  else{
    return Output;
  }
}

//Computes the integral with integral clamping(i.e turns off integral when not needed)
bool ClampIntegral(double ClampedOutput, double UnClampedOutput){
  //If Saturated is false then it's not saturated
  //if Equal signs is true then input then intgeral is getting worse
  
   bool Saturated = !(ClampedOutput == UnClampedOutput); //If values are equal then it's saturated 
   bool EqualSigns = ((ClampedOutput > 0) && (UnClampedOutput > 0)) || ((ClampedOutput < 0) && (UnClampedOutput < 0)); //If signs are not equal then EqualSigns is zero.

   
   if(Saturated && EqualSigns){//If Saturated and Equalsigns then clamp integral
    return true;
   }
   else{
    return false;
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


//Actuator Command Left
void motorRunLeft(){
  
  if(rotationLeft > InputLeft){//EncoderCount needs to increase
     moveMotorLeft('F');
  }
  else if(rotationLeft < InputLeft){//Meaning the EncoudeCount Over shoot
    moveMotorLeft('B');
  }
  else{
    moveMotorLeft('S');
  }
}

//Actuator Command Right
void motorRunRight(){
  
  if(rotationRight > InputRight){//EncoderCount needs to increase
     moveMotorRight('F');
  }
  else if(rotationRight < InputRight){//Meaning the EncoudeCount Over shoot
    moveMotorRight('B');
  }
  else{
    moveMotorRight('S');
  }
}
void setup(){
  Serial.begin(9600);
  Serial.println("Basic PIDTest:");

  //Motor Setup
  pinMode(A1A,OUTPUT);// define pin as output for Left Motor
  pinMode(A1B,OUTPUT);
  pinMode(B1A,OUTPUT);// define pin as output for Right Motor
  pinMode(B1B,OUTPUT);
  Serial.begin(9600);
  motorSpeedLeft = 0; //Set initialspeed to zero
  motorSpeedRight = 0;

  //PulseCounter Setup
  motorLeft.setInitConfig();
  motorLeft.init();
  motorRight.setInitConfig();
  motorRight.init();
  
  //PID set up
  byte  User_Setpoint_Left = 180; //User's desired rotation
  byte  User_Setpoint_Right = 180; //User's desired rotation
  
  rotationLeft = map(User_Setpoint_Left, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
  rotationRight = map(User_Setpoint_Right, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
  CutoffFreqLeft = 8000; //Select Cutoff Frequency
  CutoffFreqRight = 8000;
  SetpointLeft = rotationLeft;
  SetpointRight = rotationRight;
  InputLeft = 0;//Input is number of pulses
  InputRight = 0;
  OutputLeft = 0; //Initial value of output
  OutputRight = 0;
  delay(2000);
  //while(!Serial);
  //Serial.println("Max,Min, ACR,SR,eR");
}

void loop() {
  // put your main code here, to run repeatedly
  /*
   Serial.print("400");
    Serial.print(" ");

     Serial.print("0");
    Serial.print(" ");
    
    //Serial.print(PulseCount);
   // Serial.print(" ");
    
   // Serial.print(derivative);
    //Serial.print(" ");
    /*
    Serial.print(error);
    Serial.print(" ");

    Serial.print(Input);
    Serial.print(" ");

    Serial.print(motorSpeed);
    Serial.print(" ");
    
    Serial.print(Output);
    Serial.print(" ");
    *//*
      Serial.print(400);
    Serial.print(" ");
      Serial.print(-200);
    Serial.print(" ");
    
    Serial.print(ActuatorCommandRight);
    Serial.print(" ");
    
    
    Serial.print(SetpointRight);
    Serial.print(" ");
    
    Serial.println(errorRight);
    */
   // Serial.print(InputRight);
    //Serial.print(" ");
   
    
  InputLeft =  motorLeft.read(); //Update Input
  InputRight =  motorRight.read(); //Update Input
  
  Compute_PID();
  
  delay(SampleTime); //Add delay before computing PID
  
  motorSpeedLeft = abs(ActuatorCommandLeft); //Control speed of motor
  motorSpeedRight = abs(ActuatorCommandRight);
  
  motorRunLeft(); //Selects direction of motor
  motorRunRight();
}
