
//LeftMotor
const int A1A = 5;
const int A1B = 6;

//RightMotor
const int B1A = 7;
const int B1B = 8;

/***************************************Homing Variables***************************************/
const int HomePinLeftMotor = 16;
const int HomePinRightMotor = 17;
const byte HomingSpeed = 160;

bool HomedLeft = 0, HomedRight = 0;
byte debounce_delay = 20; //Wait 20ms for bounce on button to settle
/***********************************************************************************************/
void setup() {
  // put your setup code here, to run once:
 //Motor Setup
  pinMode(A1A,OUTPUT);// define pin as output for Left Motor
  pinMode(A1B,OUTPUT);
  pinMode(B1A,OUTPUT);// define pin as output for Right Motor
  pinMode(B1B,OUTPUT);

  //Homing Setup
  pinMode(HomePinLeftMotor, INPUT); 
  pinMode(HomePinRightMotor, INPUT); 

  Home();
  
}

void loop() {
  delay(2000);
// Serial.print(HomedLeft);
// Serial.println(HomedRight);
 Home();
}

void Home(){

  HomedLeft = digitalRead( HomePinLeftMotor); 
  delay(debounce_delay);
  HomedRight = digitalRead(HomePinRightMotor);
  delay(debounce_delay);
  while (!(HomedLeft && HomedRight)){ //while one of the motor is not homed loop0
     
      if(!HomedLeft && !HomedRight){
        //If left motor and right motore are not homed Make left motor and right motor run to towards home swtich; ie. Left motor spins CCW
       //Left Motor
       analogWrite(A1A,LOW);
       analogWrite(A1B,HomingSpeed);
       //Right Motor
       analogWrite(B1A,LOW);
       analogWrite(B1B, HomingSpeed);
      }
      else if (!HomedLeft && HomedRight){
        //If left motor is not homed but right is homed, only home left motor; ie Right motor spins CW
       //Left Motor
       analogWrite(A1A,LOW);
       analogWrite(A1B, HomingSpeed);
       //Right Motor
       analogWrite(B1A,LOW);
       analogWrite(B1B,LOW);
        
      }
      else if( HomedLeft && !HomedRight){
         //If left motor is homed but right motor is not, only home right motor
          //Left Motor
         analogWrite(A1A,LOW);
         analogWrite(A1B,LOW);
         //Right Motor
         analogWrite(B1A,LOW);
         analogWrite(B1B, HomingSpeed);
        
      }
      else{
        //Stop both motors, I could use an indicator here.eg while both motors are not home a light goes off
         //Left Motor  
         analogWrite(A1A,LOW);
         analogWrite(A1B,LOW);
         //Right Motor
         analogWrite(B1A,LOW);
         analogWrite(B1B,LOW);
        
      }

     
      HomedLeft = digitalRead( HomePinLeftMotor); 
      delay(debounce_delay);
      HomedRight = digitalRead(HomePinRightMotor);
      delay(debounce_delay);
        //delay(1000);
     // Serial.print(HomedLeft);
      //Serial.println(HomedRight);
  } 
  //Out of the loop meaning it's home so stop the motors
         analogWrite(A1A,LOW);
         analogWrite(A1B,LOW);
         //Right Motor
         analogWrite(B1A,LOW);
         analogWrite(B1B,LOW);
}
