#include <AFMotor.h>      // Motor Driver Shield Library
#include <Servo.h>        // Servo Library
#include <NewPing.h>      // NewPing Library for ultrasonic sensor

// Initialize motors using motor ports on the motor driver shield
AF_DCMotor motor1(1); // Motor 1
AF_DCMotor motor2(2); // Motor 2
AF_DCMotor motor3(3); // Motor 3
AF_DCMotor motor4(4); // Motor 4

// Ultrasonic sensor and servo pins
const int trigPin = 12;
const int echoPin = 13;
Servo servo;
const int servoPin = 9;

// Ultrasonic sensor configuration
const int maxDistance = 200;   // Maximum distance in cm to detect
NewPing sonar(trigPin, echoPin, maxDistance);  // Initialize NewPing

// Threshold distance in cm to detect obstacle
const int obstacleThreshold = 20;

void setup() {
  Serial.begin(9600);
  
  // Set motor speed (0-255)
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  
  // Attach servo to the pin
  servo.attach(servoPin);
  servo.write(90); // Center position
}

void loop() {
  int distanceFront = scanArea(90);  // Look straight ahead
  Serial.print("Distance Front: ");
  Serial.println(distanceFront);
  
  if (distanceFront <= obstacleThreshold) {
    int distanceLeft = scanArea(45);    // Scan left
    int distanceRight = scanArea(135);  // Scan right
    
    if (distanceLeft > distanceRight) {
      turnLeft();
    } else {
      turnRight();
    }
  } else {
    moveForward();
  }
}

// Function to move the robot forward
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// Function to stop the robot
void stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Function to turn the robot left
void turnLeft() {
  stop();
  delay(200);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
  stop();
}

// Function to turn the robot right
void turnRight() {
  stop();
  delay(200);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(500);
  stop();
}

// Function to scan area by moving the servo and getting distance
int scanArea(int angle) {
  servo.write(angle);   // Set servo to angle
  delay(500);           // Wait for servo to reach position
  int distance = sonar.ping_cm();  // Use NewPing to get distance in cm
  delay(100);
  return distance;
}
