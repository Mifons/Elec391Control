#include "QuadEncoder.h"
/*--------------Variables for Moving the Motor using L9110S--------------*/
const int B1A = 5;//define pin 8 for B1A
const int B1B = 6;//define pin 9 for B1B

int motorSpeed;
/*-----------------------------------------------------------*/


/*--------------Encoder Pulse CounterPID--------------*/
// Change these pin numbers to the pins connected to your encoder.
// Allowable encoder pins:
// 0, 1, 2, 3, 4, 5, 7, 30, 31 and 33
// Encoder on channel 1 of 4 available
// Phase A (pin0), PhaseB(pin1), 
QuadEncoder motorLeft(1, 0, 1, 1);
long rotation; //This is variable for mapping degrees into number of pulses
/*-----------------------------------------------------------*/


/*--------------Variables for Computing PID--------------*/
   unsigned long lastTime;
  double Input, Output, Setpoint;//input is in Pulse Count, output is motor speed, setpoint is in degrees
  double lastError; //errorSum sums all the error
  double error, derivative;
  double kp = 0.65, ki = 0.0, kd = 0.9;          //proportional, integral, derivative gains
  int SampleTime = 20; //Default Sample Rate
  
   //Clamping Output
  #define MaxClamp  200
  #define MinClamp  -200
  double ActuatorCommand; // Clamps output (Sets limit to output) initial value is 0
  
  //Clamping Integral
  bool IntegralClamp = false; //initial value is 0 so it doesn't clamp integral (turn off) in the beginning
  double integral;

  //Low Pass Filter
  double CutoffFreq; //rad/s
  //double LowPassFilterOut;
  double filtered_derivative;

  byte  User_Setpoint; //User's selected setpoint
/*-----------------------------------------------------------*/

void Compute_PID(){

  unsigned long nowTime = millis();//Obtain time first when computing PID
  //Calculate Change in time
  double delta_time = double(nowTime - lastTime);

    error = Setpoint - Input;
  
      
    if(IntegralClamp){ //Switch on or off integral clamping
      integral += error; //Integral computation
    }
    else{
      integral = 0;
    }
    
    derivative = (error-lastError); //Derivative computation

    //Low pass Filter
    filtered_derivative = derivative*CutoffFreq/(derivative+CutoffFreq);
    
    //Output
    Output = (kp*error + ki* integral + kd*filtered_derivative);

    //Clamp Output
    ActuatorCommand = ClampOutput(MinClamp, MaxClamp);

    //Clamp Integral
    IntegralClamp = ClampIntegral(ActuatorCommand, Output);
  
  
  //Store variables for next computation of PID
  lastError = error;
  lastTime = nowTime;
}

//For Choosing Gains
void Choose_Gains (double Kp, double Ki, double Kd){
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

//This is for clamping the output signal (setting limits to output signal) before going into the actuator(in this case the motor controller)
double ClampOutput(double Min, double Max){
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

//Low Pass Filter
double Low_Pass_Filter(double Prev_LowPass_Out, double CutoffFreq){

    
}

//Motor L9110S function
void moveMotor(char d)
{
    if(d =='F'){
      analogWrite(B1A,LOW);
      analogWrite(B1B,motorSpeed); 
    }else if(d =='B'){
      analogWrite(B1A,motorSpeed);
      analogWrite(B1B,LOW);    
    }else{
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF      
      analogWrite(B1A,LOW);
      analogWrite(B1B,LOW);     
    }

}// move end 

//Actuator Command
void motorRun(){
  
  if(rotation > Input){//EncoderCount needs to increase
     moveMotor('F');
  }
  else if(rotation < Input){//Meaning the EncoudeCount Over shoot
    moveMotor('B');
  }
  else{
    moveMotor('S');
  }
}

void setup(){
  Serial.begin(9600);
  Serial.println("Basic PIDTest:");

  //Motor Setup
  pinMode(B1A,OUTPUT);// define pin as output
  pinMode(B1B,OUTPUT);
  Serial.begin(9600);
  motorSpeed = 0;

  //PulseCounter Setup
  motorLeft.setInitConfig();
  motorLeft.init();
  
  //PID set up
  byte  User_Setpoint = 180; //User's desired rotation
  rotation = map(User_Setpoint, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
  Choose_Gains(kp, ki, kd);//Set gains
  CutoffFreq = 8000;
  Setpoint = rotation;
  Input = 0;//Input is number of pulses
  Output = 0; //Initial value of output


}

void loop() {
  // put your main code here, to run repeatedly
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
    */

    Serial.print(integral);
    Serial.print(" ");
    
    Serial.print(Setpoint);
    Serial.print(" ");
    
    //Serial.print(error);
    //Serial.print(" ");
    //Serial.print(Input);
    //Serial.print(" ");
    
    Serial.println(Input); //Setpoint
    
  Input =  motorLeft.read(); //Update Input
  Compute_PID();
  delay(SampleTime); //Add delay before computing PID
  motorSpeed = abs(ActuatorCommand);
  motorRun();
}
