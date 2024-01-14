#include <Stepper.h>

// Define the number of steps per revolution for the stepper motor
int stepsPerRevolution = 600;

// Define the motor interface type for the line-following motors
#define motorInterfaceType 1

// Define pins for line-following motors
#define LNA 2
#define leftMotor1 3
#define leftMotor2 4

#define rightMotor1 5
#define rightMotor2 6
#define LNB 7

// Define pins for line-following sensors
#define leftSensor A1
#define rightSensor A0

// Define pins for ultrasonic sensor
#define trigPin 9
#define echoPin 8

// Define pins for stepper motor control
#define stepperMotorPin1 10
#define stepperMotorPin2 11
#define stepperMotorPin3 12
#define stepperMotorPin4 13
  
// Initialize the Stepper library for the stepper motor
Stepper myStepper(stepsPerRevolution, stepperMotorPin1, stepperMotorPin2, stepperMotorPin3, stepperMotorPin4);

// PID parameters for line following
double kp = 5.0;  // Proportional gain
double ki = 0.1;  // Integral gain
double kd = 1.0;  // Derivative gain

// Variables for PID control
int previousError = 0;
int integral = 0;

// Function to control line-following motors
void move(int leftSpeed, int rightSpeed) {
  analogWrite(leftMotor1, leftSpeed);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor1, rightSpeed);
  analogWrite(rightMotor2, 0);
}

// Function to follow the line using PID control
void followLine() {
  int sensorValueLeft = analogRead(leftSensor);
  int sensorValueRight = analogRead(rightSensor);
  pinMode(LNA, HIGH);
  pinMode(LNB, HIGH);

  // Calculate the error for PID control
  int error = sensorValueLeft - sensorValueRight;

  // Update the integral term
  integral += error;

  // Calculate PID control output
  int pidOutput = kp * error + ki * integral + kd * (error - previousError);

  // Update previous error for the next iteration
  previousError = error;

  // Adjust motor speeds based on PID control output
  int leftSpeed = 255 - pidOutput;
  int rightSpeed = 255 + pidOutput;


  // Keep motor speeds within valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Move the robot
  move(leftSpeed, rightSpeed);
}

// Function to measure distance using the ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) / 58;  // Convert pulse duration to distance (cm)
}

void setup() {
  // Set up the line-following motors
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  // Set up the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up the stepper motor
  myStepper.setSpeed(60);  // You can adjust the speed as needed
 
}

void loop() {
  // Follow the line using PID control
  followLine();

  // Check for the presence of a box
  int distanceToBox = getDistance();
  
  // If a box is detected, slow down and stop
  if (distanceToBox > 0 && distanceToBox <= 2) { 
    
    // Slow down the line-following motors
    move(100, 100);
    
        
    // Lift the box using the stepper motor
    myStepper.step(stepsPerRevolution);
    stepsPerRevolution = -stepsPerRevolution;
   

    // You can add additional actions or code here when the box is detected
    

  }

  // You can add a delay to control the loop frequency
  delay(10);
}

