#ifndef PID_H
#define PID_H

struct PIDConfig {
  float kp;
  float ki;
  float kd;
  float deltaT;
  float setpoint;
  float tolerance;
};

struct PIDState {
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  float error = 0.0;
  float prevError = 0.0;
  float delta = 0.0;
};

void pidUpdate(float actual, PIDState& state, const PIDConfig& conf);

#endif
