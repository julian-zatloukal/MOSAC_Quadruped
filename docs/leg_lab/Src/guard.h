/* Where a joint may be commanded: the intersection of the pulse range, a travel
 * cap around the start, the range measured by hand, and (tibia, when the
 * calibrator pose is known) the linkage model. */
#ifndef LEG_LAB_GUARD_H
#define LEG_LAB_GUARD_H

#include <stdbool.h>

typedef struct {
    float       lo, hi;         /* command degrees */
    const char *lo_src, *hi_src;
} window_t;

bool     guard_has_ref(int n);
bool     guard_has_hand(int n);
float    guard_q_of_pot(int n, float pot_deg);   /* needs the calibrator pose */
float    guard_pot_of_q(int n, float q);
float    guard_hand(int n, int hi);
float    guard_ref(int n);

/* Window for joint j (servo n) around `start` (command deg). `offset` = pot - command. */
window_t guard_window(int n, int j, float start, float travel, float offset);
void     guard_print(int n, const window_t *w, float offset);

#endif
