#include <QTRSensors.h>

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;  // Keep 1 for black line, 0 for white line
//-----------------------------------------

// PID variables
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;
int currentSpeed = 40;
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };
int activeSensors;
float Kp = 0.06;
float Kd = 0.8;
float Ki = 0.00001;

int onLine = 1;

// QTR Sensor setup
QTRSensors qtr;
const uint8_t SensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
uint16_t sensorValues[8];

void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  // Pushbutton
  pinMode(12, INPUT_PULLUP);  // Pushbutton
  pinMode(13, OUTPUT);        // LED

  pinMode(5, OUTPUT);      // Standby for older carrier boards
  digitalWrite(5, HIGH);   // Enables the motor driver

  // Configure QTR sensor
  qtr.setTypeRC();  // Set sensor type to RC (reflectance)
  qtr.setSensorPins(SensorPins, 8);

  // Calibrate QTR sensor
  calibrate();
}

void loop() {
  // Wait for button press to start
  while (digitalRead(11)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  // PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error < 1000) {
        motor1run(-50);
        motor2run(50);
      } else if (error > -1000) {
        motor1run(50);
        motor2run(-50);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  // Calculate error using QTR sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);
  error = position - 3500;  // 3500 is the center of the line

  // PID calculations
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Adjust motor speeds
  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  // Constrain motor speeds
  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, -70, 255);

  // Run motors
  motor1run(rsp);
  motor2run(lsp);
}

void calibrate() {
  // Calibrate QTR sensor
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(10);
  }

  // Stop motors after calibration
  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;

  // Read QTR sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Check if the robot is on the line
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > 500) {
      onLine = 1;
      break;
    }
  }
}

//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
