#include "motion.h"
#include <math.h>

#define MIN_DURATION_MS 100

void motion_plan(motion_t *m, const float from[3], const float to[3], float max_speed_deg_s, uint32_t now_ms)
{
  float largest = 0.0f;
  for (uint8_t s = 0; s < 3; s++)
  {
    m->from[s] = from[s];
    m->to[s] = to[s];
    float d = fabsf(to[s] - from[s]);
    if (d > largest)
      largest = d;
  }
  // Smoothstep peaks at 1.5x the average speed
  uint32_t ms = (uint32_t)(1.5f * largest / max_speed_deg_s * 1000.0f);
  m->duration_ms = ms < MIN_DURATION_MS ? MIN_DURATION_MS : ms;
  m->t_start = now_ms;
}

bool motion_sample(const motion_t *m, uint32_t now_ms, float out[3])
{
  uint32_t elapsed = now_ms - m->t_start;
  if (elapsed >= m->duration_ms)
  {
    for (uint8_t s = 0; s < 3; s++)
      out[s] = m->to[s];
    return true;
  }
  float u = (float)elapsed / (float)m->duration_ms;
  float k = u * u * (3.0f - 2.0f * u);
  for (uint8_t s = 0; s < 3; s++)
    out[s] = m->from[s] + (m->to[s] - m->from[s]) * k;
  return false;
}
