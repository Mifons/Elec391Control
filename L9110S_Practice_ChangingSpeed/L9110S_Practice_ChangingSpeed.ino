
/*
 * Display number on seven segment display with Arduino
 * Written by Ahmad Shamshiri for Robojax.com
Date modified: Jun 11, 2018 at 17:20 at Ajax, Ontario, Canada
watch video instruction for this code:https://youtu.be/-g6Q9lSHDzg

Code is provided for free AS IS without warranty. 
You can share this acode as long as you keep the above note intact.
 */



const int B1A = 5;//define pin 8 for B1A
const int B1B = 6;//define pin 9 for B1B

int motorSpeed;


void setup() {
  pinMode(B1A,OUTPUT);// define pin as output
  pinMode(B1B,OUTPUT);
  Serial.begin(9600);
  motorSpeed = 150;
  
 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    // send an intro:
  Serial.println("Move me");
  Serial.println();
  
}


  
void loop() {
  if (Serial.available() > 0) {
      char thisChar = Serial.read();
      Serial.println(thisChar);
     
      moveMotor(thisChar);
      
  }
}



/*
 * @move
 * activation rotation of motor B
 * d is the direction
 * R = Right
 * L = Left
 */
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
