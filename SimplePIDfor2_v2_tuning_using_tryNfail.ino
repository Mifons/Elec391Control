/*--------------Variables for Moving the Motor using L9110S--------------*/
#include "QuadEncoder.h"

//LeftMotor
const int A1A = 5;
const int A1B = 6;

//RightMotor
const int B1A = 7;
const int B1B = 8;

/*-----------------------------------------------------------*/

/*--------------Encoder Pulse CounterPID--------------*/
// Change these pin numbers to the pins connected to your encoder.
// Allowable encoder pins:
// 0, 1, 2, 3, 4, 5, 7, 30, 31 and 33
// Encoder on channel 1 of 4 available
// Phase A (pin0), PhaseB(pin1), 
QuadEncoder motorLeft(1, 0, 1, 1); //Left motor
QuadEncoder motorRight(2, 2, 3, 1);//Right motor

/*-----------------------------------------------------------*/

/*--------------Variables for Computing PID--------------*/
  
  double InputLeft = 0, OutputLeft = 0, SetPointLeft = 0,DerivativeLeft = 0, IntegralLeft = 0, PreviousErrorLeft = 0, PID_OutputLeft = 0;
  double InputRight = 0, OutputRight = 0, SetPointRight = 0,DerivativeRight = 0, IntegralRight = 0, PreviousErrorRight = 0, PID_OutputRight = 0;

  long PulseCountLeft = 0, PulseCountRight = 0; 
  double ActuatorCommandLeft = 0, ActuatorCommandRight = 0; // Clamps output (Sets limit to output) initial value is 0
 
  unsigned long PreviousTimeLeft = 0;
  unsigned long PreviousTimeRight = 0;

  const double kp_L = 18, ki_L =0.124, kd_L = 0.497;//a little overshoot
  //const double kp_L = 1.6000003, ki_L =0.0000004, kd_L = 0.0275;//a little overshoot
  // const double kp_L = 0.6000003, ki_L =0.0000004, kd_L = 0.00675;//No overshoot, a pretty good guess
 // const double kp_L = 1.80000009, ki_L =0.0012000006, kd_L = 0.000675;//works okay, but not optimize
  const double kp_R = 0, ki_R = 0, kd_R = 0;   
  //  const double kp_L = 2.145, ki_L = 175.9, kd_L = -0.009881;  //The initial guess I got from matlab, yeah it's bad, need more data sets I thinkg
  
  int SampleTime = 10; //Default Sample Rate milliseconds
  int SampleCount = 0;
  byte  User_Setpoint_Left = 90; //User's desired rotation
  byte  User_Setpoint_Right = 90; //User's desired rotation

  
  
   //Clamping Output
  #define MaxClamp  200
  #define MinClamp -200

  

  

  //Low Pass Filter
  double CutoffFreqLeft = 8000, CutoffFreqRight = 5000; //rad/s

/*-----------------------------------------------------------*/

//This is for clamping the output signal (setting limits to output signal) before going into the actuator(in this case the motor controller)


//Computes the integral with integral clamping(i.e turns off integral when not needed)
double ClampIntegral(double ClampedOutput, double UnClampedOutput, double PastIntegral, double NowIntegral){
  //If Saturated is false then it's not saturated
  //if Equal signs is true then input then intgeral is getting worse
  //NowIntegral is the integral calulcated without clamping
  //PastIntegral is the integral from previous pid calculation
  
   bool Saturated = !(ClampedOutput == UnClampedOutput); //If values are equal then it's not saturated; ie. do not clamp
   bool EqualSigns = ((ClampedOutput > 0) && (UnClampedOutput > 0)) || ((ClampedOutput < 0) && (UnClampedOutput < 0)); //If signs are not equal then EqualSigns is zero; ie. the integral is not getting worse

   
   if(Saturated && EqualSigns){//If Saturated and Equalsigns then clamp integral, the new integral value is the same as previous integral value
    return PastIntegral;
   }
   else{
    return NowIntegral;
   }
}


 //Motor L9110S function for Left Motor
void motorRun(double setpoint, long pulse_count, int pin1, int pin2, double motor_speed){
  
  if(setpoint > pulse_count){//EncoderCount needs to increase 
     analogWrite(pin1,LOW);
     analogWrite(pin2,motor_speed);
  }
  else if(setpoint < pulse_count){//Meaning the EncoudeCount Over shoot
     analogWrite(pin1,motor_speed);
     analogWrite(pin2,LOW);
  }
  else{
     analogWrite(pin1,LOW);
     analogWrite(pin2,LOW);  
  }
}

//This is for clamping the output signal (setting limits to output signal) before going into the actuator(in this case the motor controller)
double ClampOutput(double Output){
  if (Output > MaxClamp){
    return MaxClamp;
  }
  else if(Output < MinClamp){
    return MinClamp;
  }
  else{
    return Output;
  }
}

//Computes PID: kp, ki, kd are the gain values. input is the pulse count, prevErr is the error calculation from previous calculation, setpoint is the desired rotation, integral is the area under the error curve, 
//prev_time is needed for calculating elapsed time, prev_error is used for calculating derivative, actuator_command is the output of pid
//Use pointers so that you can update different variables that are used inside the parethesis,using only one function; eg. if we were not to use pointers, we wouldn't be able to update say integrals for left motor and right motor using only one compute pid function, we'd need two similar pid function.
void Compute_PID( double kp, double ki, double kd, double input, double setpoint, double cutoff_freq, double *derivative, double *total_integral, unsigned long *prev_time, double *prev_error, double *actuator_command){
  
  double integral_past = *total_integral; //Store total value of integral from previous calculation
  double error_past = *prev_error; //Store error from previous calculation
  unsigned long time_past = *prev_time; //Store time from previous calculation
  
  double error = (setpoint - input); // Calculate error: setpoit minus the pulse count(input)
  unsigned long time_now = millis(); //Obtain intitial time
  
  double delta_time = time_now - time_past; //Calculate change in time, used for calculating derivative and integral
  double unclamped_integral = integral_past + ( (error)* delta_time ); //Calculate integeral
  double unfiltered_derivative = (error-error_past)*delta_time;//Calculate deriavtive
  
  //Low pass Filter
  double  filtered_derivative = unfiltered_derivative * cutoff_freq / (unfiltered_derivative + cutoff_freq); //This equation is from Brian Douglas, it has something to do with laplace transform and block-diagram reduction
  
  
  double unclamped_output = kp*(error) + ki*(unclamped_integral) + kd*(filtered_derivative);//Calculate output of PID
  double clamped_output = ClampOutput(unclamped_output); //Clamp the output; ie. limit the output
  
  double clamped_integral = ClampIntegral(clamped_output, unclamped_output, integral_past, unclamped_integral);//True or false,to clamp or not to clamp

  double new_unclamped_output = kp*(error) + ki*(clamped_integral) + kd*(filtered_derivative);

  *derivative = filtered_derivative;
  *total_integral = clamped_integral;
  *actuator_command = ClampOutput(new_unclamped_output);
  *prev_error = error; //Store error to use for next calculation; ie. update previous error
  *prev_time = time_now;
}

void setup() {
  Serial.begin(9600);

  
   //Motor Setup
  pinMode(A1A,OUTPUT);// define pin as output for Left Motor
  pinMode(A1B,OUTPUT);
  pinMode(B1A,OUTPUT);// define pin as output for Right Motor
  pinMode(B1B,OUTPUT);



  //PulseCounter Setup
  motorLeft.setInitConfig();//For Left Motor Encoder
  motorLeft.init();
  motorRight.setInitConfig();//For Right Motor Encoder
  motorRight.init();


    //PID set up
     long rotationLeft = map(User_Setpoint_Left, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
     long rotationRight = map(User_Setpoint_Right, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
   
       SetPointLeft = rotationLeft;
       SetPointRight = rotationRight;
       
 delay(5000);

 
 Serial.println(PulseCountLeft);
// Serial.print(",");
 //Serial.println(SampleCount);
       
       delay(SampleTime);

 
}


//For checking that motor runs and encoder reads pulses correctly
//long positionLeft  = -999;
void loop() {
  
  PulseCountLeft = motorLeft.read();// Counts number of pulses from left motor encoder
  PulseCountRight = motorRight.read();// Counts number of pulses from left motor encoder
  
  //Compute PID
  Compute_PID( kp_L, ki_L, kd_L,  PulseCountLeft, SetPointLeft, CutoffFreqLeft, &DerivativeLeft, &IntegralLeft, &PreviousTimeLeft, &PreviousErrorLeft,  &PID_OutputLeft); //Left Motor
  Compute_PID( kp_R, ki_R, kd_R,  PulseCountRight, SetPointRight, CutoffFreqRight, &DerivativeRight, &IntegralRight, &PreviousTimeRight, &PreviousErrorRight,  &PID_OutputRight); // Right Motorw

  //Clamp output of PID
  ActuatorCommandLeft = abs(PID_OutputLeft);//Motor input(speed) needs to be positive value
  ActuatorCommandRight = abs(PID_OutputRight);//Motor input(speed) needs to be positive value


  //Input PID output to run left motor
  motorRun(SetPointLeft, PulseCountLeft, A1A, A1B, ActuatorCommandLeft);
  motorRun(SetPointRight, PulseCountRight, B1A, B1B, ActuatorCommandRight);

  /*//For checking that motor runs and encoder reads pulses correctly
  long newLeft;
  if (Serial.available() > 0) {
      char thisChar = Serial.read();
     
      moveMotorLeft(thisChar);
      
  }

 
  
   if (PulseCountLeft != positionLeft) {
    Serial.print("Left = ");
    Serial.println(PulseCountLeft);
    
    
    positionLeft = PulseCountLeft;
  }
 */
// SampleCount = SampleCount + SampleTime;
 Serial.println(PulseCountLeft);
// Serial.print(",");
 //Serial.println(SampleCount);

  delay(SampleTime); //Without delay ,the serial print for pulse count freezes
}
