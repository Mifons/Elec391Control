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
  double errorSum, lastError; //errorSum sums all the error
  double error, derivative;
  double kp = 1, ki = 0.005, kd = 0.001;          //proportional, integral, derivative gains

  //Clamping Output
  #define MaxClamp  200
  #define MinClamp  -200
/*-----------------------------------------------------------*/

void Compute_PID(){

  unsigned long nowTime = millis();//Obtain time first when computing PID
  //Calculate Change in time
  double delta_time = double(nowTime - lastTime);

  //Computations
  error = Setpoint - Input;
  errorSum += error * delta_time; // Integral computation
  derivative = (error-lastError)/ delta_time; //Derivative computation

  //Output
  Output = (kp*error + ki*errorSum + kd*derivative);
  if(Output > MaxClamp){
    Output = MaxClamp;
  }
  else if(Output < MinClamp){
    Output = MinClamp;
  }
  
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
  Setpoint = rotation;
  Input = motorLeft.read();//Input is number of pulses
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

    Serial.print(Setpoint);
    Serial.print(" ");
    
    Serial.print(error);
    Serial.print(" ");
    
    Serial.println(Output); //Setpoint
    
  Input =  motorLeft.read(); //Update Input
  Compute_PID();
  delay(20); //Add delay before computing PID
  motorSpeed = abs(Output);
  motorRun();
}
