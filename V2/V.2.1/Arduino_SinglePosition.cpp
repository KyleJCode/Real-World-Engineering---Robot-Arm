/* 
Author: Carson Baker
Date Last Modified: 1/26/2025
Description: Communicates with Python program to rotate motors accordingly
Input: Joint angles (Py)
Output: Motor manipulation w/ AccelStepper, send completion message to Python
Improvements
  1. Include more motor pins (MicroStepping, Enable)
  2. Multiple positions/joint orientation
  3. Update motor steps per revolution, gear ratio, microstepping accordingly
  4. Tune motor max speed and acceleration
*/

// Libraries
#include <Arduino.h> // For Platformio communication
#include <AccelStepper.h> // AccelStepper for multi-motor-control

// Define Motor Pins 
// Define motor 1 pins
#define dirPin1 2 // Direciton pin for motor 1
#define stepPin1 3 // Step pin for motor 1

// Define motor 2 pins
#define dirPin2 4 // Direciton pin for motor 2
#define stepPin2 5 // Step pin for motor 2

// Define motor 3 pins
#define dirPin2 6 // Direciton pin for motor 2
#define stepPin2 7 // Step pin for motor 2

// Define motor 4 pins
#define dirPin2 8 // Direciton pin for motor 2
#define stepPin2 9 // Step pin for motor 2

// Define motor 5 pins
#define dirPin2 10 // Direciton pin for motor 2
#define stepPin2 11 // Step pin for motor 2

// Define motor 6 pins
#define dirPin2 12 // Direciton pin for motor 2
#define stepPin2 13 // Step pin for motor 2

// Create an array of AccelStepper objects for the motors
const int JOINTS = 2; // Total Number of joints
AccelStepper steppers[JOINTS] = {
  AccelStepper(AccelStepper::DRIVER, stepPin1, dirPin1), // Motor 1 (step pin 2, dir pin 3)
  AccelStepper(AccelStepper::DRIVER, stepPin2, dirPin2) // Motor 2 (step pin 4, dir pin 5)
  /*
  AccelStepper(AccelStepper::DRIVER, stepPin3, dirPin3), // Motor 3 (step pin 6, dir pin 7)
  AccelStepper(AccelStepper::DRIVER, stepPin4, dirPin4), // Motor 4 (step pin 8, dir pin 9)
  AccelStepper(AccelStepper::DRIVER, stepPin5, dirPin5), // Motor 5 (step pin 10, dir pin 11)
  AccelStepper(AccelStepper::DRIVER, stepPin6, dirPin6)  // Motor 6 (step pin 12, dir pin 13)
  */
};


// Define Constants for motor steps
// Motor 1 Step Constants
const int stepsPerRev1 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor1 = 1; // If microstepping used
const int gearFactor1 = 1; // Gear ratio
const float stepPerDeg1 = stepsPerRev1 * microSteppingFactor1 * gearFactor1 / 360.0; // Steps per degree
// Motor 2 Step Constants
const int stepsPerRev2 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor2 = 1; // If microstepping used
const int gearFactor2 = 1; // Gear ratio
const float stepPerDeg2 = stepsPerRev2 * microSteppingFactor2 * gearFactor2 / 360.0; // Steps per degree

// Motor 3 Step Constants
const int stepsPerRev3 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor3 = 1; // If microstepping used
const int gearFactor3 = 1; // Gear ratio
const float stepPerDeg3 = stepsPerRev3 * microSteppingFactor3 * gearFactor3 / 360.0; // Steps per degree

// Motor 4 Step Constants
const int stepsPerRev4 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor4 = 1; // If microstepping used
const int gearFactor4 = 1; // Gear ratio
const float stepPerDeg4 = stepsPerRev4 * microSteppingFactor4 * gearFactor4 / 360.0; // Steps per degree

// Motor 5 Step Constants
const int stepsPerRev5 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor5 = 1; // If microstepping used
const int gearFactor5 = 1; // Gear ratio
const float stepPerDeg5 = stepsPerRev5 * microSteppingFactor5 * gearFactor5 / 360.0; // Steps per degree

// Motor 6 Step Constants
const int stepsPerRev6 = 200; // Full steps for the MS17HD6P4150 motor (Nema17)
const int microSteppingFactor6 = 1; // If microstepping used
const int gearFactor6 = 1; // Gear ratio
const float stepPerDeg6 = stepsPerRev6 * microSteppingFactor6 * gearFactor6 / 360.0; // Steps per degree

const float stepPerDeg[JOINTS] = {stepPerDeg1, stepPerDeg2}; //, degPerStep2}; //, degPerStep3, degPerStep4, degPerStep5, degPerStep6};

// Initialize variables to store joint angles and target steps
double jointAngles[JOINTS];
long targetSteps[JOINTS];
long targetStepsRound[JOINTS];


void setup() {
  // Start serial communication
  Serial.begin(9600);
  
  while (!Serial) {
    ;  // Wait for serial port to be available
  }

    // Configure each stepper motor
  for (int i = 0; i < JOINTS; i++) {
    steppers[i].setMaxSpeed(500); // Set maximum speed (adjust as needed)
    steppers[i].setAcceleration(100); // Set acceleration (adjust as needed)
  }
}

void loop() {
  // Check if data is available from Python
  if (Serial.available() > 0) {
    // Step 1: Read the incoming data (floats) from Python
    String input = Serial.readStringUntil('\n'); // Read incoming data
    input.trim(); // Remove any leading or trailing whitespace

    // FIX, gives both joint angles [0]
    // Step 2: Parse input into joint angles
    int numAngles = 0;
    char inputArray[input.length() + 1];
    input.toCharArray(inputArray, input.length() + 1); // Convert String to char array
    char *token = strtok(inputArray, ","); // Split the string by ','

    while (token != nullptr && numAngles < JOINTS) {
      jointAngles[numAngles] = atof(token); // Convert token to float and store in jointAngles array
      token = strtok(nullptr, ","); // Move to the next token
      numAngles++;
    }


    // Step 3: Validate received angles match the number of joints
    if (numAngles != JOINTS) {
      Serial.println("Error: Number of angles does not match number of joints.");
      return;
    }

    // Step 4: Calculate target steps for each joint/motor
    for (int i = 0; i < JOINTS; i++) {
      targetSteps[i] = jointAngles[i] * stepPerDeg[i]; // Convert radians to steps
      targetStepsRound[i] = round(targetSteps[i]); // Round target steps to whole/integer number for motor communication
    }
    
    // Step 5: Set the target steps for each joint/motor
    for (int i = 0; i < JOINTS; i++) {
      steppers[i].moveTo(targetStepsRound[i]); // Set the target position in steps
    }

    // Step 6: Run all motors until the target position is reached
    while (true) {
      bool allMotorsAtTarget = true;

      // Runs all motors even if 
      // Iterate through each motor
      for (int i = 0; i < JOINTS; i++) {
        steppers[i].run();  // Execute one step if needed
        if (steppers[i].distanceToGo() != 0) {
          allMotorsAtTarget = false;  // Motor still needs to move
        }
      }

      // Break the loop if all motors are at their target positions
      if (allMotorsAtTarget) {
        break;
      }
    }

    // Check l: Send the result back to Python
    //Serial.println(targetStepsRound[0]);
    Serial.println("Motors are at target position (hopefully)!");
    
    // Pause before checking again
    delay(500);
  }
}
