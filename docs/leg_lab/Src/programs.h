/* The test programs. Each one is started from the idle loop (USER or 's'), runs
 * to its end, and returns; main then parks whatever is still powered. */
#ifndef LEG_LAB_PROGRAMS_H
#define LEG_LAB_PROGRAMS_H

void prog_check(void);
void prog_potcal(void);
void prog_limits(void);

#endif
