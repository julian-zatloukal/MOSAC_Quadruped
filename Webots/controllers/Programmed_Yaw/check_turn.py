"""Offline check of turn speed settings: what yaw rate they give and whether the joints can follow. No Webots needed.

    python check_turn.py                                  the settings in turn_params.json
    python check_turn.py --period 3 --lift 10 --omega 0.35    try other values (any of them)
    python check_turn.py --params other.json              another settings file
    python check_turn.py --table                          peak joint speed for a grid of periods and foot lifts

The yaw rate is omega / gait_period. The joint speed is the fastest femur or tibia target movement over 50 ms in a full speed
turn from rest (left or right, three gait cycles with the ramp-up of turn_params.json), which is what a servo has to follow;
the shoulder angle is the largest sideways tilt of a leg.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Programmed_Gait_V2"))

from turn_core import analyse_turn, load_turn_params  # noqa: E402

MAX_JOINT_SPEED = 350.0  # deg/s, the same assumption as tune_gait.py (the motor model of MOSAC_Controller: ~490 no load)
SHOULDER_RANGE = 10.0    # deg, the smaller side of the -10..+15 deg the policy in quadruped_control_rpi gives the shoulders


def check(params, settings, omega):
    """(yaw rate in deg/s, peak joint speed in deg/s, peak shoulder angle in deg) of a full speed turn at omega rad per cycle."""
    r = analyse_turn(params, dict(settings, omega_max=omega))
    return r["yaw_rate_deg_s"], r["max_joint_speed_deg_s"], r["max_shoulder_deg"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--params", help="settings file (default: turn_params.json, or $TURN_PARAMS)")
    parser.add_argument("--period", type=float, help="gait_period, s")
    parser.add_argument("--lift", type=float, help="parabola_height, mm")
    parser.add_argument("--omega", type=float, help="omega_max, rad per cycle")
    parser.add_argument("--table", action="store_true", help="grid of periods and foot lifts at omega_max")
    args = parser.parse_args()

    params, settings = load_turn_params(args.params)
    if args.period:
        params["gait_period"] = args.period
    if args.lift:
        params["parabola_height"] = args.lift
    omega = args.omega or settings["omega_max"]

    if args.table:
        periods = [4.0, 3.2, 2.7, 2.2, 1.8]
        print(f"peak joint speed in deg/s at omega_max={omega} (* = above {MAX_JOINT_SPEED:.0f}); yaw rate in the last row")
        print("lift mm \\ period s" + "".join(f"{T:8.1f}" for T in periods))
        for lift in (21.7, 16.0, 12.0, 8.0):
            row = ""
            for T in periods:
                speed = check(dict(params, gait_period=T, parabola_height=lift), settings, omega)[1]
                row += f"{speed:7.0f}{'*' if speed > MAX_JOINT_SPEED else ' '}"
            print(f"{lift:>18.1f}{row}")
        print("yaw rate deg/s    " + "".join(f"{math.degrees(omega) / T:8.1f}" for T in periods))
        return 0

    rate, speed, shoulder = check(params, settings, omega)
    print(f"gait_period {params['gait_period']} s, foot lift {params['parabola_height']} mm, omega_max {omega} rad/cycle")
    print(f"  yaw rate at full command   {rate:5.1f} deg/s  ({math.degrees(omega):.1f} deg per cycle)")
    print(f"  peak femur/tibia speed     {speed:5.0f} deg/s  (assumed servo limit {MAX_JOINT_SPEED:.0f})")
    print(f"  peak shoulder angle        {shoulder:5.1f} deg    (policy range on the RPi: -10 to +15)")
    warnings = []
    if speed > MAX_JOINT_SPEED:
        warnings.append(f"joints move faster than {MAX_JOINT_SPEED:.0f} deg/s: lengthen gait_period or lower parabola_height")
    if shoulder > SHOULDER_RANGE:
        warnings.append(f"shoulders tilt beyond {SHOULDER_RANGE:.0f} deg: lower omega_max unless the real range is wider")
    for w in warnings:
        print("  WARNING:", w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
