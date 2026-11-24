/* `make report`: the board's dry run of the walk, on the host, with the same
 * config.h: windows, joint ranges in q and servo deg, speed, duration. */
#include <stdio.h>
#include "config.h"
#include "gait.h"
#include "gait_params.h"
#include "plan.h"
#include "pose.h"
#include "robot.h"

int main(void)
{
    static const gait_params_t P = GAIT_WALK_PARAMS;
    gait_t g;
    plan_t pl;
    float  crouch[SERVO_COUNT], stand[SERVO_COUNT];
    gait_init(&g, &P);

    printf("walk: period %.3f s, stride %.1f mm, lift %.1f mm, height %.1f mm; here speed %.2f, stride %.2f, %d cycles\n",
           P.period, P.parabola_length, P.parabola_height, P.standing_height, SPEED_SCALE, STRIDE, WALK_CYCLES);
    if (!pose_legs(H_LOW_MM, 0.0f, crouch) || !pose_legs(P.standing_height, 0.0f, stand)) {
        printf("crouch or standing pose out of reach\n");
        return 1;
    }
    printf("crouch (H %.0f mm): femur q %+.1f tibia q %+.1f; standing (H %.1f mm): femur q %+.1f tibia q %+.1f\n",
           H_LOW_MM, crouch[4], crouch[3], P.standing_height, stand[4], stand[3]);

    bool ok = plan_walk(&g, SPEED_SCALE, STRIDE, WALK_CYCLES, &pl, 0);
    if (!ok) {
        printf("walk plan REFUSED: %s", pl.why);
        if (pl.why_servo >= 0) printf(" (%s at q %+.1f)", SERVO[pl.why_servo].name, pl.why_q);
        printf("\n");
        return 1;
    }
    printf("walk plan: speed %.2f%s (period %.2f s), %.1f s from start to stopped and centred (%ld ticks)\n",
           pl.speed, pl.slowed ? " (lowered for MAX_JOINT_SPEED_DEG_S)" : "", P.period / pl.speed, pl.seconds, pl.ticks);
    printf("peak joint speed %.0f deg/s over 50 ms (limit %.0f), %.0f over one tick (slew limit %.0f)\n",
           pl.peak50, MAX_JOINT_SPEED_DEG_S, pl.peak_tick, SLEW_LIMIT_DEG_S);
    printf("servo  joint  q range          servo deg range  crouch  stand\n");
    for (int n = 0; n < SERVO_COUNT; n++) {
        printf("  %s  %-5s  %+6.1f .. %+6.1f", SERVO[n].name, JOINT_NAME[SERVO[n].joint], pl.q_lo[n], pl.q_hi[n]);
        if (pose_calibrated(n))
            printf("   %6.1f .. %6.1f  %6.1f  %6.1f\n", pl.deg_lo[n], pl.deg_hi[n], pose_deg(n, crouch[n]),
                   pose_deg(n, stand[n]));
        else
            printf("   (CFG_CAL_POSE not set)\n");
    }
    return 0;
}
