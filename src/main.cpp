#include <Arduino.h>
#include <Servo.h>
#include <DFRobotMotorShield.h>
#include <pid.h>
#include "utils.h"
#include "constants.h"

DFRobotMotorShield motors;

Servo irServo;
int servoPosition = 90;

int IR1Val, IR2Val, IR3Val, IR4Val, IR5Val;
bool IR1Bool, IR2Bool, IR3Bool, IR4Bool, IR5Bool;
float IR1Norm, IR2Norm, IR3Norm, IR4Norm, IR5Norm;
float numerator, denominator;
float weightedLocation; // sensor actual location

float DELTA_TIME = 50; // milliseconds

// wheel speeds
int leftSpeed, rightSpeed;
float leftDeltaSpeed, rightDeltaSpeed;
const int NORMAL_SPEED = 150;
const int maxMotorSpeed = 255;

const PIDConfig leftWheelConfig {
  .kp = 50.0,
  .ki = 5.0,
  .kd = 15.0,
  .deltaT = DELTA_TIME * MILLISEC_TO_SEC,
  .setpoint = 3,
};
PIDState leftWheelState;
const PIDConfig rightWheelConfig {
  .kp = 50.0,
  .ki = 5.0,
  .kd = 15.0,
  .deltaT = DELTA_TIME * MILLISEC_TO_SEC,
  .setpoint = 3,
};
PIDState rightWheelState;

long currentMillis = 0;
long previousMillis = 0;

int counter = 0;
bool passingLine = false;
long passedTime = 0;
int i = 0;

void setup() {
  Serial.begin(9600);

  irServo.attach(PIN_DISTANCE_SERVO);
  irServo.write(servoPosition);

  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);
  pinMode(PIN_IR3, INPUT);
  pinMode(PIN_IR4, INPUT);
  pinMode(PIN_IR5, INPUT); 

  pinMode(PIN_LINE_LED, OUTPUT);
}

void loop() {
  currentMillis = millis();
  
  IR1Val = analogRead(PIN_IR1);
  IR2Val = analogRead(PIN_IR2);
  IR3Val = analogRead(PIN_IR3);
  IR4Val = analogRead(PIN_IR4);
  IR5Val = analogRead(PIN_IR5);

  IR1Norm = computeNormVal(IR1Val, IR1_MIN, IR1_MAX);
  IR2Norm = computeNormVal(IR2Val, IR2_MIN, IR2_MAX);
  IR3Norm = computeNormVal(IR3Val, IR3_MIN, IR3_MAX);
  IR4Norm = computeNormVal(IR4Val, IR4_MIN, IR4_MAX);
  IR5Norm = computeNormVal(IR5Val, IR5_MIN, IR5_MAX);

  numerator = ((1 - IR1Norm) * 1 + (1 - IR2Norm) * 2 + (1 - IR3Norm) * 3 + (1 - IR4Norm) * 4 + (1 - IR5Norm) * 5);
  denominator = (1 - IR1Norm) + (1 - IR2Norm) + (1 - IR3Norm) + (1 - IR4Norm) + (1 - IR5Norm);
  weightedLocation = numerator / denominator;

  if (IR1Norm > 0.9 && IR2Norm > 0.9 && IR3Norm > 0.9 && IR4Norm > 0.9 && IR5Norm > 0.9) {
    digitalWrite(PIN_LINE_LED, LOW);
    if (!passingLine && (millis() - passedTime) >= 500) {
      counter++;
      passingLine = true;
      passedTime = millis();
    }
  } else {
    digitalWrite(PIN_LINE_LED, HIGH);
    passingLine = false;
  }

  if ((currentMillis - previousMillis >= DELTA_TIME)) {
    pidUpdate(weightedLocation, leftWheelState, leftWheelConfig);
    pidUpdate(weightedLocation, rightWheelState, rightWheelConfig);

    leftSpeed = constrain(NORMAL_SPEED - leftWheelState.delta, maxMotorSpeed, -maxMotorSpeed);
    rightSpeed = constrain(NORMAL_SPEED + rightWheelState.delta, maxMotorSpeed, -maxMotorSpeed);
    
    motors.setM1Speed(rightSpeed);
    motors.setM2Speed(leftSpeed);
    previousMillis = currentMillis;
  }
}