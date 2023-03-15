#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

float normalize(float sensorVal, float minVal, float maxVal) {
  return constrain((sensorVal - minVal) / (maxVal - minVal), 0, 1);
}

#endif