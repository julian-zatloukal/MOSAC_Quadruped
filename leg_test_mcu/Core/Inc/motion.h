#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

// Synchronized move of the 3 joints of a leg, in servo degrees.
// Smoothstep profile (zero speed at both ends); duration chosen so that no joint exceeds the speed limit.
typedef struct {
  float from[3];
  float to[3];
  uint32_t t_start;
  uint32_t duration_ms;
} motion_t;

void motion_plan(motion_t *m, const float from[3], const float to[3], float max_speed_deg_s, uint32_t now_ms);
bool motion_sample(const motion_t *m, uint32_t now_ms, float out[3]);  // returns true once the move is complete

#endif
