#include <Arduino.h>

static const int motorPin = 5;          // PWM output pin, connected to the motor
static const int encoderPin = A2;        // Encoder input pin, for measuring speed
static const int encoderInterrupt = 2;    // Encoder interrupt pin (should be changed according to hardware connection)
volatile unsigned int encoderCount = 0;   // Records the encoder pulse count

// PID control parameters
double kp = 0.1; // Proportional coefficient
double ki = 0.01; // Integral coefficient
double kd = 0.01; // Derivative coefficient

int pwmOutput = 0;      // PWM output value
int setpointRPM = 800; // Target RPM
int currentRPM = 0;     // Current RPM
double prevError = 0;   // Previous error
double integral = 0;    // Integral term

void setup() {
  pinMode(motorPin, OUTPUT);    // Set the motor pin as output
  pinMode(encoderPin, INPUT);   // Set the encoder pin as input
  attachInterrupt(encoderInterrupt, countEncoder, RISING); // Configure encoder interrupt

  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Calculate the current RPM
  unsigned long currentTime = millis();
  static unsigned long previousTime = 0;
  unsigned long elapsedTime = currentTime - previousTime;
 
  if (elapsedTime >= 1000) {
    currentRPM = (encoderCount * 60000) / (elapsedTime * 12); // Conversion from encoder pulses to RPM
    previousTime = currentTime;
    encoderCount = 0;
  }

  // Calculate PID control output
  int error = setpointRPM - currentRPM;
  integral += error;
  double derivative = (error - prevError) / elapsedTime;
  pwmOutput = kp * error + ki * integral + kd * derivative;
 
  // Limit the PWM output range
  if (pwmOutput > 255) {
    pwmOutput = setpointRPM / 6.67;
  } else if (pwmOutput < 0) {
    pwmOutput = 75;
  }

  // Control the motor
  analogWrite(motorPin, pwmOutput);

  // Output information
  currentRPM = pwmOutput* 6.67;
  if(abs(currentRPM-setpointRPM) <= 200){
    Serial.print("Current RPM: ");
    Serial.print(currentRPM);
  }
  Serial.print(" | Setpoint RPM: ");
  Serial.print(setpointRPM);
  Serial.print(" | PWM Output: ");
  Serial.println(pwmOutput);

  prevError = error;
}

void countEncoder() {
  encoderCount++;
}
