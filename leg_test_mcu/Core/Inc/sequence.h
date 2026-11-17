#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdint.h>

// One pose of the test sequence: offsets from home in the leg frame (deg), then a pause
typedef struct {
  const char *label;
  float body;
  float femur;
  float tibia;
  uint16_t hold_ms;
} waypoint_t;

extern const waypoint_t SEQUENCE[];
extern const uint8_t SEQUENCE_LENGTH;

#endif
