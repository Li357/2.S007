#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DFRobotMotorShield.h>
#include <pid.h>
#include "utils.h"
#include "constants.h"

DFRobotMotorShield motors;
Adafruit_BNO055 bnoIMU(55);

Servo irServo;
int servoPosition = 90;

int IR1Val, IR2Val, IR3Val, IR4Val, IR5Val;
float IR1Norm, IR2Norm, IR3Norm, IR4Norm, IR5Norm;
float weightedLocation;

float DELTA_TIME_PID = 50;
float DELTA_TIME_LINE = 500;

int leftSpeed, rightSpeed;
float leftDeltaSpeed, rightDeltaSpeed;
const int NORMAL_SPEED = 100;
const int maxMotorSpeed = 255;

enum State {
  A, B, C, D, E,
  IDLE, TURNING,
};
State nextPos = A;
State currentState = IDLE;

const PIDConfig leftWheelConfig {
  .kp = 30.0,
  .ki = 0.0,
  .kd = 0.0,
  .deltaT = DELTA_TIME_PID * MILLISEC_TO_SEC,
  .setpoint = 3,
  .tolerance = 0,
};
PIDState leftWheelState;
const PIDConfig rightWheelConfig {
  .kp = 30.0,
  .ki = 0.0,
  .kd = 0.0,
  .deltaT = DELTA_TIME_PID * MILLISEC_TO_SEC,
  .setpoint = 3,
  .tolerance = 0,
};
PIDState rightWheelState;

const PIDConfig turningLeftWheelConfig {
  .kp = 50.0,
  .ki = 5.0,
  .kd = 15.0,
  .deltaT = DELTA_TIME_PID * MILLISEC_TO_SEC,
  .setpoint = 90,
  .tolerance = 1,
};
PIDState turningLeftWheelState;
const PIDConfig turningRightWheelConfig {
  .kp = 50.0,
  .ki = 5.0,
  .kd = 15.0,
  .deltaT = DELTA_TIME_PID * MILLISEC_TO_SEC,
  .setpoint = 90,
  .tolerance = 1,
};
PIDState turningRightWheelState;

sensors_event_t imuEvent;

long currentMillis = 0;
long previousMillis = 0;

int dx = 0;
int dy = 0;
int dxTarget = 0;
int dyTarget = 0;
bool passingLine = false;
long passedLineMillis = 0;

float yawAngle = 0.0;

double remapAngle(double angle) {
  angle = angle * DEG_TO_RAD;
  return RAD_TO_DEG * (atan2(cos(angle), -sin(angle)));
}

void setup() {
  Serial.begin(9600);

  if (!bnoIMU.begin()) {
    Serial.print("No BNO055 detected :(");
    while (1);
  }

  bnoIMU.setExtCrystalUse(true);
  bnoIMU.getEvent(&imuEvent);
  yawAngle = remapAngle(imuEvent.orientation.x);

  irServo.attach(PIN_DISTANCE_SERVO);
  irServo.write(servoPosition);

  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);
  pinMode(PIN_IR3, INPUT);
  pinMode(PIN_IR4, INPUT);
  pinMode(PIN_IR5, INPUT); 

  pinMode(PIN_LINE_LED, OUTPUT);

  delay(500);
  currentState = A;
  nextPos = B;
}

void loop() {
  if (currentState == IDLE) {
    motors.setSpeeds(0, 0);
    return;
  }

  // we first reach Y-dest then turn to start moving in X-dir
  if (dy > 0 && dy >= dyTarget && currentState != TURNING) {
    currentState = TURNING;
  }

  //Serial.println(currentState);

  currentMillis = millis();
  
  IR1Val = analogRead(PIN_IR1);
  IR2Val = analogRead(PIN_IR2);
  IR3Val = analogRead(PIN_IR3);
  IR4Val = analogRead(PIN_IR4);
  IR5Val = analogRead(PIN_IR5);

  IR1Norm = 1 - normalize(IR1Val, IR1_MIN, IR1_MAX);
  IR2Norm = 1 - normalize(IR2Val, IR2_MIN, IR2_MAX);
  IR3Norm = 1 - normalize(IR3Val, IR3_MIN, IR3_MAX);
  IR4Norm = 1 - normalize(IR4Val, IR4_MIN, IR4_MAX);
  IR5Norm = 1 - normalize(IR5Val, IR5_MIN, IR5_MAX);

  weightedLocation = (IR1Norm * 1 + IR2Norm * 2 + IR3Norm * 3 + IR4Norm * 4 + IR5Norm * 5) / (IR1Norm + IR2Norm + IR3Norm + IR4Norm + IR5Norm);

  if (IR1Norm < 0.1 && IR2Norm < 0.1 && IR3Norm < 0.1 && IR4Norm < 0.1 && IR5Norm < 0.1) {
    digitalWrite(PIN_LINE_LED, LOW);
    if (!passingLine && (millis() - passedLineMillis) >= DELTA_TIME_LINE) {
      (dy < dyTarget ? dy : dx)++;
      passingLine = true;
      passedLineMillis = millis();
    }
  } else {
    digitalWrite(PIN_LINE_LED, HIGH);
    passingLine = false;
  }

  if ((currentMillis - previousMillis >= DELTA_TIME_PID)) {
    if (currentState == TURNING) {
      bnoIMU.getEvent(&imuEvent);
      yawAngle = remapAngle(imuEvent.orientation.x);

      pidUpdate(yawAngle, turningLeftWheelState, turningLeftWheelConfig);
      pidUpdate(yawAngle, turningRightWheelState, turningRightWheelConfig);

      if (turningLeftWheelState.error == 0 || turningRightWheelState.error == 0) {
        currentState = nextPos;
        nextPos = nextPos == B ? C : nextPos == C ? D : nextPos == D ? E : IDLE;
        return;
      }

      leftSpeed = constrain(NORMAL_SPEED - turningLeftWheelState.delta, maxMotorSpeed, -maxMotorSpeed);
      rightSpeed = constrain(NORMAL_SPEED + turningRightWheelState.delta, maxMotorSpeed, -maxMotorSpeed);
    } else {
      if (currentState == A) {
        dxTarget = 0;
        dyTarget = 12;
      } else if (currentState == B) {
        dxTarget = 13;
        dyTarget = 0;
      } else if (currentState == C) {
        dxTarget = 0;
        dyTarget = 11;
      } else if (currentState == D) {
        dxTarget = 0;
        dyTarget = 13;
      }

      pidUpdate(weightedLocation, leftWheelState, leftWheelConfig);
      pidUpdate(weightedLocation, rightWheelState, rightWheelConfig);

      leftSpeed = constrain(NORMAL_SPEED - leftWheelState.delta, maxMotorSpeed, -maxMotorSpeed);
      rightSpeed = constrain(NORMAL_SPEED + rightWheelState.delta, maxMotorSpeed, -maxMotorSpeed);

      Serial.println(leftSpeed);
      Serial.println(rightSpeed);

      motors.setM1Speed(rightSpeed);
      motors.setM2Speed(leftSpeed);
    }
    //motors.setSpeeds(rightSpeed, leftSpeed);
    previousMillis = currentMillis;
  }
}