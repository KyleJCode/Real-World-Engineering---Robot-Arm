// Pin definitions
const int stepPin = 3;  // STEP pin connected to DRV8825
const int dirPin = 2;   // DIR pin connected to DRV8825

void setup() {
  // Set pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Set initial direction
  digitalWrite(dirPin, HIGH); // HIGH = one direction, LOW = reverse direction
}

void loop() {
  // Rotate the motor at a constant speed
  digitalWrite(stepPin, HIGH); // Generate a step pulse
  delayMicroseconds(1000);     // Adjust speed: Higher delay = slower rotation
  digitalWrite(stepPin, LOW);  // Complete the step pulse
  delayMicroseconds(1000);     // Same delay for consistent speed
}
