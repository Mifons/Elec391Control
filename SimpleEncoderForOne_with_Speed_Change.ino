/* Teensy 4 H/S Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include "QuadEncoder.h"


const int B1A = 7;//define pin 8 for B1A
const int B1B = 8;//define pin 9 for B1B

int motorSpeed;


// Change these pin numbers to the pins connected to your encoder.
// Allowable encoder pins:
// 0, 1, 2, 3, 4, 5, 7, 30, 31 and 33
// Encoder on channel 1 of 4 available
// Phase A (pin0), PhaseB(pin1), 
QuadEncoder motorLeft(1, 2, 3, 1);


//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  pinMode(B1A,OUTPUT);// define pin as output
  pinMode(B1B,OUTPUT);
  motorSpeed = 255;
  Serial.println("One Knob Encoder Test:");
  /* Initialize Encoder/motorLeft. */
  motorLeft.setInitConfig();
  motorLeft.init();
}

long positionLeft  = -999;


void loop() {
  long newLeft;
  newLeft = motorLeft.read();
  if (newLeft != positionLeft) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    char thisChar = Serial.read();
    Serial.println("Reset motor to zero");
    motorLeft.write(0);     
    moveMotor(thisChar);
  }
}
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
