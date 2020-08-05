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
  
  double InputLeft = 0, OutputLeft = 0, DerivativeLeft = 0, IntegralLeft = 0, PreviousErrorLeft = 0, PID_OutputLeft = 0;
  double InputRight = 0, OutputRight = 0, DerivativeRight = 0, IntegralRight = 0, PreviousErrorRight = 0, PID_OutputRight = 0;
 
  unsigned long PreviousTimeLeft;
  unsigned long PreviousTimeRight;
  
   const double kp_L = 18.0, ki_L = 0.15, kd_L = 0.5;//proportional, integral, derivative gains
   const double kp_R = 40, ki_R = 0.3, kd_R = 0.00002;  

  

 //Acceptable error for the travel function
 int AccErr = 3;
  //  const double kp_L = 2.145, ki_L = 175.9, kd_L = -0.009881;  //The initial guess I got from matlab, yeah it's bad, need more data sets I thinkg
  
  int SampleTime = 10; //Default Sample Rate: Sample per milliseconds

  
   //Clamping Output
  #define MaxClamp  200
  #define MinClamp -200

  //Need to reset this back to zero after the Home function
   long PulseCountLeft;
   long PulseCountRight;
  
/*--------------Variables for Travel Function--------------*/
const int TravelDurationLeft = 500; //How long the PID will compute; this allows the arm to settle in on the set point, instead of overshooting it once it reaches the set point because of momentum  
const int TravelDurationRight = 1000; //The duration for left motor is different to right motor because all motors home once right reaches its set point. I may not even need the TraveDurationRight
/*-----------------------------------------------------------*/
 
  
/*--------------Low Pass Filter Variables--------------*/
  double CutoffFreqLeft = 5000, CutoffFreqRight = 5000; //rad/s

/*-----------------------------------------------------------*/

/***************************************Homing Variables***************************************/
const int HomePinLeftMotor = 16;
const int HomePinRightMotor = 15;
const byte HomingSpeed = 250;
const byte HoldSpeed = 0;

bool HomedLeft = 0, HomedRight = 0;
byte debounce_delay = 20; //Wait 20ms for bounce on button to settle
/***********************************************************************************************/


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
//prev_time is needed for calculating elapsed time, prev_error is used for calculating derivative, atuator_command is the output of pid
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

/*--------------Moves arm to a designated setpoint--------------*/
void Travel(long left_new_setpoint, long right_new_setpoint ){
    
   //PID set up
     long rotationLeft = map(left_new_setpoint, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count
     long rotationRight = map(right_new_setpoint, 0, 360, 0 , 400); //Maps User's setpoint from degrees to pulse count

  //For how long to keep computing PID when traveling to setpoint
     unsigned long timeCheck_1 = 0;
     unsigned long timeCheck_2 = 0;
     unsigned long timePassed = 0;
   
   
   //Work with the motors individually to avoid the case where the xylophone stick glides on the instrument
     double SetPointLeft = rotationLeft;
     double SetPointRight = 0;

    PulseCountLeft = 0;
    PulseCountRight = 0;
       
  double ActuatorCommandLeft = 0, ActuatorCommandRight = 0; // Clamps output (Sets limit to output) initial value is 0
 
  //First Move the Left Motor as it's the one that Travels to the location of the note. Otherwise, if the right button reaches the setpoint first, the xylophone stick will glide on the notes on the way to the left motor's setpoint
  //Serial.print(PulseCountLeft);
 //Serial.print(";");
//  Serial.println(SetPointLeft-AccErr);


  while( (timePassed < TravelDurationLeft) ){//Exit loop if Pulse Count falls with in a range around the Setpoint

    // timeCheck_1 = millis();
     PulseCountLeft = motorLeft.read();
     PulseCountRight = motorRight.read();
   
     //Comput PID
     Compute_PID( kp_L, ki_L, kd_L,  PulseCountLeft, SetPointLeft, CutoffFreqLeft, &DerivativeLeft, &IntegralLeft, &PreviousTimeLeft, &PreviousErrorLeft,  &PID_OutputLeft); 
     Compute_PID( kp_R, ki_R, kd_R,  PulseCountRight, SetPointRight, CutoffFreqRight, &DerivativeRight, &IntegralRight, &PreviousTimeRight, &PreviousErrorRight,  &PID_OutputRight); // Right Motorw
     
     //Motor input(speed) needs to be positive value
     ActuatorCommandLeft = abs(PID_OutputLeft);//Motor input(speed) needs to be positive value
     ActuatorCommandRight = abs(PID_OutputRight);//Motor input(speed) needs to be positive value
     
       //Input the PID output to the motor
     motorRun(SetPointLeft, PulseCountLeft, A1A, A1B, ActuatorCommandLeft);
     motorRun(SetPointRight, PulseCountRight, B1A, B1B, ActuatorCommandRight);

          //Counts number of pulses from left motor encoder
     //PulseCountLeft = motorLeft.read();
    // PulseCountRight = motorRight.read(); //Because of friction, we need to make sure the right motor stay in place while the left motor rotates, so we calculate the right motor PID.

     delay(SampleTime);

    // timeCheck_2 = millis();

     timePassed = timePassed + SampleTime;

   //  Serial.println(timePassed);
     
  }

  //delay(1000);
  //Change Setpoint for right motor, this time the setpoint for the left motor stays the same so the xylphone stick can hit a note consistenly. Also the speed of the motors are different from each other, so it's better to work with each motor individually.
  SetPointRight = rotationRight;
  //SetPointLeft =  motorLeft.read();
  
  
 while(SetPointRight != PulseCountRight){

     //Counts number of pulses from left motor encoder
     PulseCountLeft = motorLeft.read();
     PulseCountRight = motorRight.read(); //Because of friction, we need to make sure the right motor stay in place while the left motor rotates, so we calculate the right motor PID.

     //Comput PID
     Compute_PID( kp_L, ki_L, kd_L,  PulseCountLeft, SetPointLeft, CutoffFreqLeft, &DerivativeLeft, &IntegralLeft, &PreviousTimeLeft, &PreviousErrorLeft,  &PID_OutputLeft); 
     Compute_PID( kp_R, ki_R, kd_R,  PulseCountRight, SetPointRight, CutoffFreqRight, &DerivativeRight, &IntegralRight, &PreviousTimeRight, &PreviousErrorRight,  &PID_OutputRight); // Right Motorw
     
     //Motor input(speed) needs to be positive value
     ActuatorCommandLeft = abs(PID_OutputLeft);//Motor input(speed) needs to be positive value
     ActuatorCommandRight = abs(PID_OutputRight);//Motor input(speed) needs to be positive value
     
     //Input the PID output to the motor
     motorRun(SetPointLeft, PulseCountLeft, A1A, A1B, ActuatorCommandLeft);
     motorRun(SetPointRight, PulseCountRight, B1A, B1B, ActuatorCommandRight);

     
     delay(SampleTime);
  }   

  ActuatorCommandLeft = 0;
  ActuatorCommandRight = 0;
  
  motorRun(SetPointLeft, PulseCountLeft, A1A, A1B, ActuatorCommandLeft);
  motorRun(SetPointRight, PulseCountRight, B1A, B1B, ActuatorCommandRight);

  delay(10);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*--------------Home for moving arm back to home position and reseting PID calculation--------------*/
void Home(){
    
     HomedLeft = digitalRead( HomePinLeftMotor); 
      delay(debounce_delay);
     HomedRight = digitalRead(HomePinRightMotor);
      delay(debounce_delay);
      
      while (!HomedLeft || !HomedRight){ //while one of the motor is not homed loop
         
          
           analogWrite(A1A,HomingSpeed);
           analogWrite(A1B,LOW);
           
           //Right Motor
           analogWrite(B1A, HomingSpeed);
           analogWrite(B1B,LOW);
        
         
             delay(500); //The arm bounces so much when it contacts with thw switch so need to wait for it to settle
          HomedLeft = digitalRead( HomePinLeftMotor); 
          delay(debounce_delay);
          HomedRight = digitalRead(HomePinRightMotor);
          delay(debounce_delay);
        
          
      } 
    
      //Out of the loop meaning it's home so stop the motors
             analogWrite(A1A,LOW);
             analogWrite(A1B, LOW);
             //Right Motor
             analogWrite(B1A,LOW);
             analogWrite(B1B, LOW);

             //Reset Starting point for pulse count; eg, when you first travel, it will remember how much pulse count, so need to reset it when returning home
             delay(500); //this delay allows the arms to bounce away from the switches when the motors are off
             motorLeft.init();
             motorRight.init();
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


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

  //Initialize Pulse Count
  PulseCountLeft = 0;
  PulseCountRight = 0;
   
     
       
       Home();
       
}


//For checking that motor runs and encoder reads pulses correctly
//long positionLeft  = -999;
void loop() {
  //delay(1000);
 Travel(230,0); //Plays the note, input are in degrees
 Home();

 while(1){
 
// Serial.println("Done");
 }
 
 //delay(1000);
}
