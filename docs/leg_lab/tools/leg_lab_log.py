#!/usr/bin/env python3
"""Record and summarise leg_lab serial output.

  python leg_lab_log.py record --port COM5 --out fr_limits.log
      Copies the ST-LINK serial port to the screen and to a file (needs pyserial). Ctrl-C stops.
      Any serial terminal that saves a log works just as well.

  python leg_lab_log.py summary fr_limits.log [--json fr.json] [--plot fr.png] [--gait]
      Reads the RES,... lines of one or more logs and prints, per servo: the limits in servo degrees, pulse
      widths and (when the calibrator pose was set) the joint angle q of the Webots model; the femur x tibia
      and femur x body maps; the foot positions they allow; and with --gait, whether the Programmed_Gait_V2
      gait (gait_params.json) stays inside the measured ranges of that leg.

  python leg_lab_log.py selftest
      Checks the forward kinematics used here against gait_core.servo_angles.

q convention: servo deg = mid point + dir * q, the same as quadruped_control_rpi/main.py; at the printed
calibrator q = (0, -42.65021, -18.34979) for (body, femur, tibia), as main.h DELTA_REF_*.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GAIT_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Webots", "controllers", "Programmed_Gait_V2"))
REF_Q = {"body": 0.0, "femur": -42.65021, "tibia": -18.34979}
LEG_INDEX = {"BL": 0, "FL": 1, "BR": 2, "FR": 3}   # gait_core leg order


# --------------------------------------------------------------------------------------------- parsing

def parse(paths):
    d = {"meta": {}, "window": {}, "limit": [], "map": [], "check": [], "potfit": [], "potcal": [],
         "hand": {}, "ref": {}}
    for path in paths:
        with open(path, errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line.startswith("RES,"):
                    continue
                p = line.split(",")
                kind = p[1]
                try:
                    if kind == "meta":
                        d["meta"][p[3]] = {"leg": p[2], "joint": p[5], "dir": int(p[7]), "ref": float(p[9]),
                                           "hand": (float(p[11]), float(p[12]))}
                    elif kind == "window":
                        d["window"][p[3]] = {"leg": p[2], "start": float(p[5]), "offset": float(p[7]),
                                             "lo": float(p[9]), "lo_src": p[10], "hi": float(p[12]), "hi_src": p[13]}
                    elif kind == "limit":
                        e = {"leg": p[2], "servo": p[3], "ctx": p[4], "femur": float(p[6]), "side": p[7],
                             "kind": p[8], "cmd": float(p[10]), "us": int(p[12]), "pot": float(p[14]),
                             "repeat": float(p[16])}
                        if len(p) > 18 and p[17] == "q":
                            e["q"] = float(p[18])
                        d["limit"].append(e)
                    elif kind == "map":
                        d["map"].append({"leg": p[2], "servo": p[3], "femur": float(p[5]), "lo_kind": p[7],
                                         "lo": float(p[8]), "hi_kind": p[10], "hi": float(p[11])})
                    elif kind == "check":
                        d["check"].append({"leg": p[2], "servo": p[3], "status": p[4], "start": float(p[6]),
                                           "offset": float(p[8]), "gain": float(p[10]), "backlash": float(p[12]),
                                           "track": float(p[14])})
                    elif kind == "potfit":
                        d["potfit"].append({"servo": p[2], "a": float(p[4]), "b": float(p[6]), "resid": float(p[8]),
                                            "hyst": float(p[10]), "table_offset": float(p[12]),
                                            "monotonic": p[14] == "1"})
                    elif kind == "potcal":
                        d["potcal"].append({"servo": p[2], "deg": float(p[3]), "up": float(p[4]), "down": float(p[5])})
                    elif kind == "hand":
                        d["hand"][p[3]] = (float(p[4]), float(p[5]))
                    elif kind == "ref":
                        d["ref"][p[3]] = float(p[4])
                except (IndexError, ValueError):
                    print(f"skipped a malformed line: {line}", file=sys.stderr)
    return d


def q_of_pot(meta, pot):
    """Joint angle q (deg) of a pot reading, or None without a calibrator pose."""
    if meta is None or meta["ref"] < -900:
        return None
    return REF_Q[meta["joint"]] + (pot - meta["ref"]) * meta["dir"]


def q_of_cmd(d, servo, cmd):
    w = d["window"].get(servo)
    return q_of_pot(d["meta"].get(servo), cmd + (w["offset"] if w else 0.0))


# ------------------------------------------------------------------------------------------ kinematics

def load_gait_core():
    sys.path.insert(0, GAIT_DIR)
    import gait_core  # noqa: E402
    return gait_core


def foot_position(gc, q_femur_deg, q_tibia_deg):
    """Forward kinematics of one leg, inverse of gait_core.servo_angles: (H, D1) in mm, or None."""
    phi_femur = math.radians(q_femur_deg + gc.MID_POINT_SOLID_FEMUR)
    phi_servo = math.radians(q_tibia_deg + gc.MID_POINT_SOLID_TIBIA)
    A, B, C, D, DELTA, EPS = gc.A, gc.B, gc.C, gc.D, gc.DELTA, gc.EPSILON

    def servo_of_w(w):
        a1 = (C * C + w * w - D * D) / (2 * C * w)
        a2 = (A * A + w * w - B * B) / (2 * A * w)
        if abs(a1) > 1 or abs(a2) > 1:
            return None
        return math.pi - DELTA - math.acos(a1) - math.acos(a2)

    # W between |B-A| and C+D; phi_servo(W) is monotonic on the branch the model uses.
    lo, hi = abs(B - A) + 1e-9, C + D - 1e-9
    f_lo, f_hi = servo_of_w(lo), servo_of_w(hi)
    if f_lo is None or f_hi is None or not (min(f_lo, f_hi) <= phi_servo <= max(f_lo, f_hi)):
        return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if (servo_of_w(mid) - phi_servo) * (f_lo - phi_servo) > 0:
            lo, f_lo = mid, servo_of_w(mid)
        else:
            hi = mid
    w = 0.5 * (lo + hi)
    x = math.acos(max(-1.0, min(1.0, (C * C + D * D - w * w) / (2 * C * D))))
    phi_tibia = math.pi + DELTA - EPS - x - gc.PSI
    knee = phi_femur + phi_tibia                       # interior knee angle
    d2 = math.sqrt(gc.L_FEMUR ** 2 + gc.L_TIBIA ** 2 - 2 * gc.L_FEMUR * gc.L_TIBIA * math.cos(knee))
    alpha = math.acos(max(-1.0, min(1.0, (d2 * d2 + gc.L_FEMUR ** 2 - gc.L_TIBIA ** 2) / (2 * gc.L_FEMUR * d2))))
    theta = math.pi / 2 - phi_femur - alpha            # = atan(D1 / H)
    return d2 * math.cos(theta), d2 * math.sin(theta)


def gait_trajectory(gc, params_path):
    sys.path.insert(0, GAIT_DIR)
    import gait_config  # noqa: E402
    p = dict(gait_config.DEFAULTS)
    if params_path and os.path.exists(params_path):
        with open(params_path) as f:
            p.update(json.load(f))
    g = gc.GaitGenerator(p)
    out = []
    for _ in range(int(2 * p["gait_period"] / g.dt)):
        g.step()
        t = list(g.targets)
        out.append([(math.degrees(t[2 * k]), math.degrees(t[2 * k + 1])) for k in range(4)])
    return p, out


# ---------------------------------------------------------------------------------------------- report

def fmt(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"


def summary(args):
    d = parse(args.logs)
    leg = next(iter({m["leg"] for m in d["meta"].values()}), None) or \
        next(iter({e["leg"] for e in d["limit"] + d["check"]}), None)
    out = {"leg": leg, "servos": {}, "map": {}, "check": d["check"], "potfit": d["potfit"],
           "hand": d["hand"], "ref": d["ref"]}

    if d["hand"]:
        print("hand ranges (pot deg): " + ", ".join(f"{s} {lo:.1f}..{hi:.1f}" for s, (lo, hi) in d["hand"].items()))
    if d["ref"]:
        print("calibrator pose (pot deg): " + ", ".join(f"{s} {v:.2f}" for s, v in d["ref"].items()))
    for c in d["check"]:
        print(f"check {c['servo']}: {c['status']}, offset {c['offset']:+.1f}, gain {c['gain']:.2f}, "
              f"backlash {c['backlash']:+.1f}, tracking {c['track']:.1f}")
    for p in d["potfit"]:
        print(f"potcal {p['servo']}: adc = {p['a']:.1f} + {p['b']:.3f}*deg, residual {p['resid']:.2f} deg, "
              f"hysteresis {p['hyst']:.2f}, stored table off by {p['table_offset']:+.1f} deg"
              f"{'' if p['monotonic'] else ', NOT MONOTONIC'}")

    starts = [e for e in d["limit"] if e["ctx"] == "start"]
    if starts:
        print(f"\nlimits of the {leg} leg (command deg = what to send; us; q = Webots joint angle):")
        print(f"  {'servo':6}{'side':6}{'cmd':>8}{'us':>7}{'q':>8}  kind (repeat)")
        for e in starts:
            q = q_of_cmd(d, e["servo"], e["cmd"])
            rep = f" ({e['repeat']:.1f})" if e["repeat"] >= 0 else ""
            print(f"  {e['servo']:6}{e['side']:6}{e['cmd']:8.1f}{e['us']:7d}{fmt(q):>8}  {e['kind']}{rep}")
            s = out["servos"].setdefault(e["servo"], {})
            s[e["side"]] = {"cmd": e["cmd"], "us": e["us"], "q": q, "kind": e["kind"], "repeat": e["repeat"]}
        for servo, w in d["window"].items():
            out["servos"].setdefault(servo, {})["window"] = w
            out["servos"][servo]["meta"] = d["meta"].get(servo)

    # maps: the femur position of the 1-D search counts as a map point too
    femur_servo = next((s for s, m in d["meta"].items() if m["joint"] == "femur"), None)
    for other_joint in ("tibia", "body"):
        other = next((s for s, m in d["meta"].items() if m["joint"] == other_joint), None)
        if other is None:
            continue
        rows = {}
        for m in d["map"]:
            if m["servo"] == other:
                rows[round(m["femur"], 1)] = (m["lo"], m["lo_kind"], m["hi"], m["hi_kind"])
        lo = next((e for e in starts if e["servo"] == other and e["side"] == "low"), None)
        hi = next((e for e in starts if e["servo"] == other and e["side"] == "high"), None)
        if rows and lo and hi:
            rows[round(lo["femur"], 1)] = (lo["cmd"], lo["kind"], hi["cmd"], hi["kind"])
        if not rows:
            continue
        print(f"\nfemur x {other_joint} map ({femur_servo} -> {other} low .. high), command deg [q]:")
        table = []
        for f in sorted(rows):
            l, lk, h, hk = rows[f]
            qf, ql, qh = q_of_cmd(d, femur_servo, f), q_of_cmd(d, other, l), q_of_cmd(d, other, h)
            table.append({"femur": f, "lo": l, "hi": h, "lo_kind": lk, "hi_kind": hk, "q_femur": qf, "q_lo": ql,
                          "q_hi": qh})
            print(f"  femur {f:6.1f} [{fmt(qf):>6}]   {other} {l:6.1f} .. {h:6.1f}  [{fmt(ql):>6} .. {fmt(qh):>6}]"
                  f"   ({lk} / {hk})")
        out["map"][other_joint] = table

    gc = None
    tib = out["map"].get("tibia")
    if tib and all(r["q_femur"] is not None and r["q_lo"] is not None for r in tib):
        gc = load_gait_core()
        print("\nfoot positions at the tibia limits (H = hip to foot, D1 = foot offset, + = backwards; mm):")
        for r in tib:
            a = foot_position(gc, r["q_femur"], r["q_lo"])
            b = foot_position(gc, r["q_femur"], r["q_hi"])
            r["foot_lo"], r["foot_hi"] = a, b
            sa = f"H {a[0]:6.1f} D1 {a[1]:6.1f}" if a else "outside the model"
            sb = f"H {b[0]:6.1f} D1 {b[1]:6.1f}" if b else "outside the model"
            print(f"  femur q {r['q_femur']:6.1f}: tibia low -> {sa}   tibia high -> {sb}")
    elif tib:
        print("\n(no calibrator pose for the femur and tibia: q and foot positions need 'c' in PROGRAM_MONITOR)")

    if args.gait:
        gait_check(d, out, tib, leg, args.params)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
        print(f"\nwrote {args.json}")
    if args.plot:
        plot(out, tib, args.plot, leg, args.params if args.gait else None)


def interp(rows, qf, key):
    pts = sorted((r["q_femur"], r[key]) for r in rows)
    if qf < pts[0][0] or qf > pts[-1][0]:
        return None
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= qf <= x1:
            return y0 if x1 == x0 else y0 + (y1 - y0) * (qf - x0) / (x1 - x0)
    return pts[-1][1]


def gait_check(d, out, tib, leg, params_path):
    if not tib or any(r["q_femur"] is None or r["q_lo"] is None for r in tib):
        print("\n--gait needs the femur x tibia map with the calibrator pose set")
        return
    gc = load_gait_core()
    params, traj = gait_trajectory(gc, params_path)
    k = LEG_INDEX[leg]
    qs = [s[k] for s in traj]
    worst, outside = float("inf"), 0
    lo_side = [min(r["q_lo"], r["q_hi"]) for r in tib]
    for qf, qt in qs:
        a = interp(tib, qf, "q_lo")
        b = interp(tib, qf, "q_hi")
        if a is None or b is None:
            outside += 1
            continue
        lo, hi = min(a, b), max(a, b)
        margin = min(qt - lo, hi - qt)
        worst = min(worst, margin)
        if margin < 0:
            outside += 1
    fq = [q[0] for q in qs]
    tq = [q[1] for q in qs]
    print(f"\ngait check ({os.path.basename(params_path) if params_path else 'defaults'}, period "
          f"{params['gait_period']:.2f} s), {leg} leg:")
    print(f"  gait femur q {min(fq):.1f} .. {max(fq):.1f}, tibia q {min(tq):.1f} .. {max(tq):.1f}")
    print(f"  measured femur q {min(r['q_femur'] for r in tib):.1f} .. {max(r['q_femur'] for r in tib):.1f}, "
          f"tibia q {min(lo_side):.1f} .. {max(max(r['q_lo'], r['q_hi']) for r in tib):.1f}")
    if outside:
        print(f"  {outside} of {len(qs)} gait samples are OUTSIDE the measured ranges (or the mapped femur span)")
    else:
        print(f"  the whole gait fits; smallest margin to a tibia limit {worst:.1f} deg")
    out["gait_check"] = {"outside": outside, "samples": len(qs), "worst_margin": worst}


def plot(out, tib, path, leg, params_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed: no plot")
        return
    if not tib:
        print("no femur x tibia map to plot")
        return
    use_q = all(r["q_femur"] is not None and r["q_lo"] is not None for r in tib)
    kx, kl, kh = ("q_femur", "q_lo", "q_hi") if use_q else ("femur", "lo", "hi")
    rows = sorted(tib, key=lambda r: r[kx])
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot([r[kx] for r in rows], [r[kl] for r in rows], "o-", label="tibia low limit")
    ax.plot([r[kx] for r in rows], [r[kh] for r in rows], "s-", label="tibia high limit")
    if params_path is not None and use_q:
        gc = load_gait_core()
        _, traj = gait_trajectory(gc, params_path)
        k = LEG_INDEX[leg]
        ax.plot([s[k][0] for s in traj], [s[k][1] for s in traj], "-", lw=1, alpha=0.7, label="gait")
    unit = "q (deg, Webots)" if use_q else "command (deg)"
    ax.set_xlabel(f"femur {unit}")
    ax.set_ylabel(f"tibia {unit}")
    ax.set_title(f"{leg} leg: measured femur x tibia range")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"wrote {path}")


# ----------------------------------------------------------------------------------------------- other

def record(args):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is needed: pip install pyserial")
    with serial.Serial(args.port, args.baud, timeout=0.2) as s, open(args.out, "a", newline="") as f:
        print(f"recording {args.port} to {args.out}, Ctrl-C to stop. Keys are not sent: use the USER button.")
        try:
            while True:
                data = s.read(4096)
                if data:
                    text = data.decode(errors="replace")
                    sys.stdout.write(text)
                    sys.stdout.flush()
                    f.write(text)
                    f.flush()
        except KeyboardInterrupt:
            print("\nstopped")


def selftest(_args):
    import numpy as np
    gc = load_gait_core()
    worst = 0.0
    for H in np.linspace(120, 175, 12):
        for D1 in np.linspace(-60, 60, 13):
            t = gc.servo_angles(np.full(4, H), np.full(4, D1))
            p = foot_position(gc, math.degrees(t[0]), math.degrees(t[1]))
            if p is None:
                print(f"FK failed at H {H:.1f} D1 {D1:.1f}")
                return 1
            worst = max(worst, abs(p[0] - H), abs(p[1] - D1))
    print(f"forward kinematics vs gait_core.servo_angles: largest error {worst:.2e} mm")
    return 0 if worst < 1e-3 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--port", required=True)
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--out", default="leg_lab.log")
    s = sub.add_parser("summary")
    s.add_argument("logs", nargs="+")
    s.add_argument("--json")
    s.add_argument("--plot")
    s.add_argument("--gait", action="store_true")
    s.add_argument("--params", default=os.path.join(GAIT_DIR, "gait_params.json"))
    sub.add_parser("selftest")
    a = ap.parse_args()
    if a.cmd == "record":
        record(a)
    elif a.cmd == "summary":
        summary(a)
    else:
        sys.exit(selftest(a))


if __name__ == "__main__":
    main()
