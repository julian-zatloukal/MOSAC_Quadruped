#include "sequence.h"

/*
 * Played in a loop once the leg has reached home. Offsets are in the leg frame of the policy and
 * simulation, so the same numbers mean the same motion on every leg; the firmware applies each
 * joint's direction. Limits: body -10..+15, femur -20..+30, tibia -15..+15 (anything beyond is clipped).
 */
const waypoint_t SEQUENCE[] = {
  //  label           body  femur  tibia  hold ms
  { "home",            0.0f,  0.0f,  0.0f, 1000 },
  // one joint at a time, both directions
  { "body +10",       10.0f,  0.0f,  0.0f,  500 },
  { "body -8",        -8.0f,  0.0f,  0.0f,  500 },
  { "body home",       0.0f,  0.0f,  0.0f,  500 },
  { "femur +25",       0.0f, 25.0f,  0.0f,  500 },
  { "femur -15",       0.0f,-15.0f,  0.0f,  500 },
  { "femur home",      0.0f,  0.0f,  0.0f,  500 },
  { "tibia +12",       0.0f,  0.0f, 12.0f,  500 },
  { "tibia -12",       0.0f,  0.0f,-12.0f,  500 },
  { "tibia home",      0.0f,  0.0f,  0.0f,  500 },
  // all three together
  { "combined A",      5.0f, 20.0f,-10.0f,  500 },
  { "combined B",     -5.0f,-10.0f, 10.0f,  500 },
  { "home",            0.0f,  0.0f,  0.0f, 1500 },
};

const uint8_t SEQUENCE_LENGTH = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);
