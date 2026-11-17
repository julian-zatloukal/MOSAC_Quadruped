#include "feedback.h"
#include "board.h"
#include "calibration.h"

static volatile uint16_t dma_buffer[FEEDBACK_SCANS][3];

// Per slot: ADC counts of each calibration point, mean of the down and up tables
static float table_adc[3][CALIB_POINTS];

static uint16_t raw[3];
static float deg[3];
static bool valid[3];

void feedback_init(const uint8_t leg_joints[3])
{
  for (uint8_t s = 0; s < 3; s++)
  {
    const float *t = CALIB_TABLES[leg_joints[s]];
    for (uint8_t i = 0; i < CALIB_POINTS; i++)
      table_adc[s][i] = (t[i] + t[CALIB_POINTS + i]) * 0.5f / CALIB_SAMPLES_PER_POINT;
  }
  board_start_feedback(&dma_buffer[0][0], FEEDBACK_SCANS * 3);
}

// Median by Wirth's k-th smallest selection (partially reorders v)
static uint16_t median(uint16_t *v, int32_t n)
{
  int32_t k = n / 2, l = 0, m = n - 1;
  while (l < m)
  {
    uint16_t x = v[k];
    int32_t i = l, j = m;
    do
    {
      while (v[i] < x) i++;
      while (x < v[j]) j--;
      if (i <= j)
      {
        uint16_t t = v[i]; v[i] = v[j]; v[j] = t;
        i++;
        j--;
      }
    } while (i <= j);
    if (j < k) l = i;
    if (k < i) m = j;
  }
  return v[k];
}

// Linear interpolation in the calibration table; outside it the reading is flagged invalid
static float adc_to_deg(const float *table, float adc, bool *in_range)
{
  if (adc < table[0])
  {
    *in_range = false;
    return CALIB_ANGLES[0];
  }
  for (uint8_t i = 1; i < CALIB_POINTS; i++)
  {
    if (adc <= table[i])
    {
      *in_range = true;
      return CALIB_ANGLES[i - 1] + (CALIB_ANGLES[i] - CALIB_ANGLES[i - 1]) * (adc - table[i - 1]) / (table[i] - table[i - 1]);
    }
  }
  *in_range = false;
  return CALIB_ANGLES[CALIB_POINTS - 1];
}

void feedback_update(void)
{
  static uint16_t samples[FEEDBACK_SCANS];

  // The DMA keeps writing while we copy: harmless, the window just mixes a few newer samples
  for (uint8_t s = 0; s < 3; s++)
  {
    for (uint32_t i = 0; i < FEEDBACK_SCANS; i++)
      samples[i] = dma_buffer[i][s];
    raw[s] = median(samples, FEEDBACK_SCANS);
    deg[s] = adc_to_deg(table_adc[s], raw[s], &valid[s]);
  }
}

uint16_t feedback_raw(uint8_t slot) { return raw[slot]; }
float feedback_deg(uint8_t slot) { return deg[slot]; }
bool feedback_valid(uint8_t slot) { return valid[slot]; }
