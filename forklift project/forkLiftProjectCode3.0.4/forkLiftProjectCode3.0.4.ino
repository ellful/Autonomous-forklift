#include <SoftwareSerial.h>
#include <Stepper.h>
SoftwareSerial BT_Serial(0, 1); // RX, TX
//line follower  pins
#define enA 3//Enable1 L298 Pin enA 
#define in1 2 //Motor1  L298 Pin in1 
#define in2 4 //Motor1  L298 Pin in1 
#define in3 5 //Motor2  L298 Pin in1 
#define in4 7 //Motor2  L298 Pin in1 
#define enB 6 //Enable2 L298 Pin enB 
//sensor pins
#define R_S A1 //ir sensor Right
#define L_S A3 //ir sensor Left

// Define pins for stepper motor control
#define stepperMotorPin1 10
#define stepperMotorPin2 11
#define stepperMotorPin3 12
#define stepperMotorPin4 13
// Define pins for ultrasonic sensor
#define trigPin 9
#define echoPin 8


  // Initialize the Stepper library for the stepper motor
  int stepsPerRevolution = 600;
Stepper myStepper(stepsPerRevolution, stepperMotorPin1, stepperMotorPin2, stepperMotorPin3, stepperMotorPin4);
bool forkPos = true;
int bt_data; // variable to receive data from the serial port
int Speed = 130;  

int mode=0;

void setup(){ // put your setup code here, to run once

pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input

pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 


 // Set up the stepper motor
  myStepper.setSpeed(60);  // You can adjust the speed as needed
  
  // Set up the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

 
Serial.begin(9600); // start serial communication at 9600bps
BT_Serial.begin(9600); 
delay(500);
}


void loop(){  
  
if(BT_Serial.available() > 0){  //if some date is sent, reads it and saves in state     
bt_data = BT_Serial.read();      
if(bt_data > 20){Speed = bt_data;}      
}

     if(bt_data == 8){mode=1; Speed=130;} //Auto Line Follower Command  
else if(bt_data == 9){mode=0; Stop();}    //Manual Android Application Control Command

analogWrite(enA, Speed); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(enB, Speed); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 

if(mode==0){     
//===============================================================================
//                          Key Control Command
//=============================================================================== 

     if(bt_data == 1){forword(); }  // if the bt_data is '1' the DC motor will go forward
else if(bt_data == 2){backword();}  // if the bt_data is '2' the motor will Reverse
else if(bt_data == 3){turnRight();}  // if the bt_data is '3' the motor will turn left
else if(bt_data == 4){turnLeft();} // if the bt_data is '4' the motor will turn right
else if(bt_data == 10){stepperMoveUp();}
else if(bt_data == 11){stepperMoveDown();}
else if(bt_data == 5){Stop(); }     // if the bt_data '5' the motor will Stop


//===============================================================================
//                          Voice Control Command
//===============================================================================    
else if(bt_data == 6){turnRight();  delay(400);  bt_data = 5;}
else if(bt_data == 7){turnLeft(); delay(400);  bt_data = 5;}

//===============================================================================
//                          Stepper control commands
//===============================================================================


}else{    
//===============================================================================
//                          Line Follower Control
//===============================================================================     
if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){Stop();}  //if Right Sensor and Left Sensor are at White color then it will call forword function
if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}//if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){forword();}     //if Right Sensor and Left Sensor are at Black color then it will call Stop function

// If a box is detected, slow down and stop
 // Check for the presence of a box
  int distanceToBox = getDistance();

  if (distanceToBox > 0 && distanceToBox <= 2) { 
    
    // Slow down the line-following motors
    Stop();
    
        
    // Lift the box using the stepper motor
    if (forkPos){
      myStepper.step(-stepsPerRevolution);
      forkPos = false;

      }
      if (!forkPos){
        myStepper.step(stepsPerRevolution);
        forkPos = true;
      }
    
   

    // You can add additional actions or code here when the box is detected
    

  }

} 

delay(10);
}

void forword(){  //forword
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
}

void backword(){ //backword
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW);  //Left Motor forword Pin 
}

void turnRight(){ //turnRight
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin  
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
}

void turnLeft(){ //turnLeft
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW);  //Left Motor forword Pin 
}

void Stop(){ //stop
digitalWrite(in1, LOW); //Right Motor forword Pin 
digitalWrite(in2, LOW); //Right Motor backword Pin 
digitalWrite(in3, LOW); //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin 
}

void stepperMoveUp(){
if (forkPos){
 myStepper.step(-stepsPerRevolution);
   
   forkPos = false; //false for up
}
}

void stepperMoveDown(){
  if(!forkPos){
    myStepper.step(stepsPerRevolution);

    forkPos = true; //true is to show that the fork is in the upper position
  }
  }
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) / 58;  // Convert pulse duration to distance (cm)
}
