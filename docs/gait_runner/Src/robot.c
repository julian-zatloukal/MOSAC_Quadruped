#include "robot.h"
#include "gait.h"

/* As leg_lab/Src/robot.c (pins checked on the robot with servo_test_leg). */
const servo_info_t SERVO[SERVO_COUNT] = {
    /*  n      name   PWM     pot    IN  dir  leg     joint       */
    /*  0 */ { "BFR", "PA15", "PA4",  4, +1, LEG_FR, JOINT_BODY  },
    /*  1 */ { "BBR", "PA1",  "PC3", 13, -1, LEG_BR, JOINT_BODY  },
    /*  2 */ { "BBL", "PB10", "PA2",  2, -1, LEG_BL, JOINT_BODY  },
    /*  3 */ { "TFR", "PB11", "PC1", 11, +1, LEG_FR, JOINT_TIBIA },
    /*  4 */ { "FFR", "PC6",  "PC4", 14, +1, LEG_FR, JOINT_FEMUR },
    /*  5 */ { "BFL", "PC7",  "PA7",  7, +1, LEG_FL, JOINT_BODY  },
    /*  6 */ { "FFL", "PC8",  "PA5",  5, -1, LEG_FL, JOINT_FEMUR },
    /*  7 */ { "TFL", "PC9",  "PA6",  6, -1, LEG_FL, JOINT_TIBIA },
    /*  8 */ { "FBL", "PD12", "PC0", 10, -1, LEG_BL, JOINT_FEMUR },
    /*  9 */ { "TBL", "PD13", "PA3",  3, -1, LEG_BL, JOINT_TIBIA },
    /* 10 */ { "TBR", "PD14", "PC2", 12, +1, LEG_BR, JOINT_TIBIA },
    /* 11 */ { "FBR", "PD15", "PB1",  9, +1, LEG_BR, JOINT_FEMUR },
};

const leg_info_t LEG[LEG_COUNT] = {
    [LEG_FR] = { "front right", "FR", { 0, 4, 3 },   G_FR },
    [LEG_FL] = { "front left",  "FL", { 5, 6, 7 },   G_FL },
    [LEG_BR] = { "back right",  "BR", { 1, 11, 10 }, G_BR },
    [LEG_BL] = { "back left",   "BL", { 2, 8, 9 },   G_BL },
};

const char *const JOINT_NAME[NJ] = { "body", "femur", "tibia" };
