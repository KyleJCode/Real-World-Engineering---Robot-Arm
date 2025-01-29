#include <AccelStepper.h>

// Define the first motor pins
#define dirPin1 6  // Direction pin for Motor 1
#define stepPin1 7 // Step pin for Motor 1

// Define the second motor pins
#define dirPin2 4  // Direction pin for Motor 2
#define stepPin2 5 // Step pin for Motor 2

// Create AccelStepper objects for the two motors
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {
  // Set maximum speed and acceleration for both motors
  motor1.setMaxSpeed(1000);  // Adjust speed as needed
  motor1.setAcceleration(500); // Optional acceleration
  motor1.setSpeed(200); // Constant speed (steps per second)

  motor2.setMaxSpeed(1000);  // Same configuration for motor 2
  motor2.setAcceleration(500);
  motor2.setSpeed(500); // Constant speed
}

void loop() {
  // Move both motors at the same speed
  motor1.runSpeed();
  motor2.runSpeed();
}
