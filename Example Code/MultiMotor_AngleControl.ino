#include <AccelStepper.h>

// Define the first motor pins
#define dirPin1 2  // Direction pin for Motor 1
#define stepPin1 3 // Step pin for Motor 1

// Define the second motor pins
#define dirPin2 4  // Direction pin for Motor 2
#define stepPin2 5 // Step pin for Motor 2

// Create AccelStepper objects for the two motors
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);

// Define constants for motor steps
const int stepsPerRevolution = 200;  // Full steps for your MS17HD6P4150 motor (no microstepping)
const int microsteppingFactor = 1;   // Full-step mode
const int stepsPerFullRevolution = stepsPerRevolution * microsteppingFactor;

// Define angles for movement
float angle1 = 90;  // Rotate Motor 1 by 90 degrees
float angle2 = 90; // Rotate Motor 2 by 180 degrees

// Convert angles to steps
int stepsForMotor1 = (angle1 / 360.0) * stepsPerFullRevolution;
int stepsForMotor2 = (angle2 / 360.0) * stepsPerFullRevolution;

void setup() {
  Serial.begin(9600); // Debugging output

  delay(2000);

  // Set maximum speed and acceleration for both motors
  motor1.setMaxSpeed(500);      // Adjust speed if needed
  motor1.setAcceleration(100);  // Adjust acceleration for smoother movement

  motor2.setMaxSpeed(1200);
  motor2.setAcceleration(700);


}

void loop() {
  // Move motors to the specified positions
  motor1.moveTo(stepsForMotor1);
  motor2.moveTo(stepsForMotor2);

  // Run the motors until they reach their targets
  motor1.run();
  motor2.run();

  // Debugging output
  Serial.print("Motor 1 Distance to Target: ");
  Serial.println(motor1.distanceToGo());
  Serial.print("Motor 2 Distance to Target: ");
  Serial.println(motor2.distanceToGo());

  // Once both motors reach their positions, stop running the code
  if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
    Serial.println("Both motors have reached their target positions.");

    // Disable motor outputs to avoid holding torque and reduce power consumption
    motor1.disableOutputs();
    motor2.disableOutputs();

    // Halt further execution
    while (true) {
      // Motors have reached the target position; idle here
    }
  }
}
