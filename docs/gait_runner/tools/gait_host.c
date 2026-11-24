/* Host driver for tools/test_gait.py: runs Src/gait.c from a script on stdin.
 *   dt <s>        gait time per step (default 0.01)
 *   drive <f>     gait_drive
 *   stop          gait_stop
 *   step          gait_tick, then prints: stage idle t femur/tibia of BL FL BR FR (rad)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gait.h"
#include "gait_params.h"

int main(void)
{
    static const gait_params_t P = GAIT_WALK_PARAMS;
    gait_t g;
    gait_init(&g, &P);
    double dt = 0.01;
    char line[128];
    while (fgets(line, sizeof line, stdin)) {
        if (!strncmp(line, "dt ", 3)) {
            dt = atof(line + 3);
        } else if (!strncmp(line, "drive ", 6)) {
            gait_drive(&g, atof(line + 6));
        } else if (!strncmp(line, "stop", 4)) {
            gait_stop(&g);
        } else if (!strncmp(line, "step", 4)) {
            gait_tick(&g, dt);
            printf("%d %d %.9f", g.stage, g.idle ? 1 : 0, g.t);
            for (int k = 0; k < G_LEGS; k++) printf(" %.9g %.9g", g.femur[k], g.tibia[k]);
            printf(" %d\n", g.ik_ok ? 1 : 0);
        }
    }
    return 0;
}
