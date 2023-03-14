#include <Arduino.h>
#include "pid.h"

void pidUpdate(float current, PIDState& state, const PIDConfig& conf) {
  state.error = conf.setpoint - current;
  if (abs(state.error) < conf.tolerance) {
    state.error = 0;
  }

  state.i += state.error * conf.deltaT;
  state.d = (state.error - state.prevError) / conf.deltaT;
  state.delta = int(conf.kp * state.error + conf.ki * state.i + conf.kd * state.d);
  state.prevError = state.error;
}
