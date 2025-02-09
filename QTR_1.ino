#include <QTRSensors.h>

// Define sensor pins
#define NUM_SENSORS 8
unsigned int sensorValues[NUM_SENSORS];

// Define motor pins
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 9
#define RIGHT_MOTOR_BACKWARD 10

// PID constants
float Kp = 0.1;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.05; // Derivative gain

// Speed settings
int maxSpeed = 150; // Maximum speed of the motors
int baseSpeed = 100; // Base speed of the motors

// PID variables
int lastError = 0;
int integral = 0;

// QTR sensor object
QTRSensors qtr;
const uint8_t SensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  // Initialize motor pins as output
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Configure the QTR sensors
  qtr.setTypeRC(); // Set the sensor type to RC (reflectance)
  qtr.setSensorPins(SensorPins, NUM_SENSORS);

  // Calibrate the QTR sensors
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(10);
  }
}

void loop() {
  // Read sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate error
  int error = position - 3500; // 3500 is the ideal position (center of the line)

  // Calculate PID terms
  int proportional = error;
  integral += error;
  int derivative = error - lastError;

  // Calculate motor speed adjustment
  int adjustment = Kp * proportional + Ki * integral + Kd * derivative;

  // Adjust motor speeds
  int leftMotorSpeed = baseSpeed + adjustment;
  int rightMotorSpeed = baseSpeed - adjustment;

  // Constrain motor speeds to maxSpeed
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // Set motor speeds
  setMotorSpeed(LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, leftMotorSpeed);
  setMotorSpeed(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, rightMotorSpeed);

  // Update last error
  lastError = error;
}

void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
  if (speed > 0) {
    analogWrite(forwardPin, speed);
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -speed);
  }
}
