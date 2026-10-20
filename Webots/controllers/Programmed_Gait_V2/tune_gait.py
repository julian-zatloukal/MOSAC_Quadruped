"""Tunes the constants of Programmed_Gait_V2 by running the gait headless in Webots and searching for the best ones.

The search is scipy's differential evolution over SEARCH_SPACE (gait_config.py). Every candidate is first checked offline
(foot reach, joint range, joint speed) and only then simulated, several Webots instances in parallel. The simulation is
deterministic, so the same parameters always give the same score.

Score of a candidate that walks without falling (higher is better):
    forward speed - w_lateral*|lateral speed| - w_yaw*|heading change rate| - w_tilt*(tilt above tilt_ok)
Candidates that fall, reach out of range, move the joints faster than a servo can follow (max_joint_speed),
need more torque than the servo model delivers (max_torque) or tilt too much (tilt_limit) score below -1.

Usage (needs scipy and Webots; WEBOTS can point to the executable):
    WEBOTS=/path/to/webots python tune_gait.py --out runs/first              search, then write runs/first/best_params.json
    python tune_gait.py --evaluate gait_params.json other.json               score parameter files, no search
    python tune_gait.py --evaluate defaults                                  score the original constants
"""
import argparse
import json
import math
import multiprocessing
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

import numpy as np
from scipy.optimize import differential_evolution
from scipy.stats import qmc

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gait_config import DEFAULTS, SEARCH_SPACE, load_params  # noqa: E402
from gait_core import GaitInfeasible, analyse  # noqa: E402

DEFAULT_WORLD = os.path.join(HERE, "..", "..", "worlds", "MOSAC_REAL_V4_gait_v2.wbt")


# ---------------------------------------------------------------------------------------------------------------------
# Running one candidate
# ---------------------------------------------------------------------------------------------------------------------
def make_project(out_dir, world):
    """A Webots project folder under out_dir that links to the repo's protos and controllers. Running from it keeps
    Webots from writing its project files into the repo. The world can be any file that refers to ../protos."""
    world = os.path.abspath(world)
    repo_webots = os.path.abspath(os.path.join(HERE, "..", ".."))
    project = os.path.join(out_dir, "project")
    os.makedirs(os.path.join(project, "worlds"), exist_ok=True)
    shutil.copy(world, os.path.join(project, "worlds"))
    for sub in ("protos", "controllers"):
        link = os.path.join(project, sub)
        if not os.path.islink(link):
            os.symlink(os.path.join(repo_webots, sub), link)
    return project, os.path.join("worlds", os.path.basename(world))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def run_webots(cfg, params, measure_s=None):
    """Simulates one parameter set. Returns the controller's result dict (valid=False if the run failed)."""
    trials = os.path.join(cfg["out"], "trials")
    os.makedirs(trials, exist_ok=True)
    trial = tempfile.mkdtemp(prefix="trial_", dir=trials)
    params_path = os.path.join(trial, "params.json")
    result_path = os.path.join(trial, "result.json")
    with open(params_path, "w") as f:
        json.dump(params, f)
    env = dict(os.environ, GAIT_PARAMS=params_path, GAIT_RESULT=result_path,
               GAIT_WARMUP_S=str(cfg["warmup_s"]), GAIT_MEASURE_S=str(measure_s or cfg["measure_s"]))
    cmd = [cfg["webots"], "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr",
           f"--port={free_port()}", cfg["world"]]
    error = None
    with open(os.path.join(trial, "webots.log"), "w") as log:
        proc = subprocess.Popen(cmd, cwd=cfg["project"], env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, preexec_fn=lambda: os.nice(10))  # leave the CPU to other work
        try:
            proc.wait(timeout=cfg["timeout"])
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
            error = f"timeout after {cfg['timeout']} s"
    if os.path.exists(result_path):
        with open(result_path) as f:
            result = json.load(f)
    else:
        result = {"valid": False, "error": error or f"no result file (Webots exit code {proc.returncode})"}
    if result.get("valid") and not cfg["keep_trials"]:
        shutil.rmtree(trial, ignore_errors=True)
    else:
        result["trial_dir"] = trial  # the log of a failed run (or of every run with --keep-trials)
    return result


# ---------------------------------------------------------------------------------------------------------------------
# Checks and score
# ---------------------------------------------------------------------------------------------------------------------
def yaw_enabled(params):
    return params["yaw_kp"] > 0 or params["yaw_ki"] > 0


def offline_check(cfg, params):
    """Checks that need no simulation. Returns (penalty, message); penalty > 0 means a limit is violated."""
    penalty = 0.0
    try:
        a = analyse(params, cycles=1.0)
        if yaw_enabled(params):  # the yaw loop may stretch the stride of one side and shift the feet sideways
            for omega in (params["yaw_max"], -params["yaw_max"]):
                analyse(params, cycles=1.0, omega=omega)
    except GaitInfeasible as error:
        return 5.0, str(error)
    messages = []
    speed_excess = a["max_joint_speed_deg_s"] / cfg["max_joint_speed"] - 1
    if speed_excess > 0:
        penalty += speed_excess
        messages.append(f"joint speed {a['max_joint_speed_deg_s']:.0f} deg/s > {cfg['max_joint_speed']:.0f}")
    low = np.array(cfg["envelope_min"]) - cfg["range_margin"]
    high = np.array(cfg["envelope_max"]) + cfg["range_margin"]
    range_excess = max((low - a["joint_min_deg"]).max(), (a["joint_max_deg"] - high).max())
    if range_excess > 0:
        penalty += range_excess / 10
        messages.append(f"joint range exceeds the original's by more than {cfg['range_margin']:.0f} deg")
    return penalty, "; ".join(messages)


def score_result(cfg, r):
    """Score of a simulation result (see the module docstring). Also returns a short status."""
    if not r.get("valid"):
        return -3.0, "failed: " + r.get("error", "?")
    if r["fell"]:
        return -2.0 + 0.5 * min(1.0, r["t_gait"] / r["t_window"][1]), "fell"
    if r["torque_p99"] > cfg["max_torque"]:
        return -1.0 - min(0.9, r["torque_p99"] / cfg["max_torque"] - 1), f"torque {r['torque_p99']:.1f} N.m"
    if r["tilt_max_deg"] > cfg["tilt_limit"]:
        return -1.0 - min(0.9, (r["tilt_max_deg"] - cfg["tilt_limit"]) / 20), f"tilt {r['tilt_max_deg']:.0f} deg"
    s = r["v_forward"]
    s -= cfg["w_lateral"] * abs(r["v_lateral"])
    s -= cfg["w_yaw"] * abs(math.radians(r["yaw_change_deg"])) / r["window_s"]
    s -= cfg["w_tilt"] * max(0.0, r["tilt_max_deg"] - cfg["tilt_ok"])
    return s, "ok"


def evaluate(cfg, params, measure_s=None):
    """Full evaluation of a parameter set. Returns a dict with the score, status and the main metrics."""
    penalty, message = offline_check(cfg, params)
    if penalty > 0:
        return {"score": -1.0 - min(0.9, penalty), "status": "limit: " + message, "simulated": False}
    result = run_webots(cfg, params, measure_s)
    score, status = score_result(cfg, result)
    out = {"score": score, "status": status, "simulated": True}
    out.update({k: v for k, v in result.items() if k not in ("trace", "params")})
    if "trace" in result:
        out["trace"] = result["trace"]
    return out


class Objective:
    """What differential evolution minimises. A class so that it can be pickled for the worker processes."""

    def __init__(self, cfg, base):
        self.cfg = cfg
        self.base = base
        self.names = list(SEARCH_SPACE)

    def params(self, x):
        values = {n: (bool(v >= 0.5) if isinstance(DEFAULTS[n], bool) else float(v)) for n, v in zip(self.names, x)}
        return dict(self.base, **values)

    def __call__(self, x):
        params = self.params(x)
        start = time.time()
        out = evaluate(self.cfg, params)
        out.pop("trace", None)
        line = {"time": round(time.time() - start, 1), "params": {n: params[n] for n in self.names}, **out}
        with open(os.path.join(self.cfg["out"], "trials.jsonl"), "a") as f:  # one short line per write: safe to append
            f.write(json.dumps(line) + "\n")
        return -out["score"]


# ---------------------------------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------------------------------
def build_cfg(args):
    os.makedirs(args.out, exist_ok=True)
    project, world = make_project(args.out, args.world)
    baseline = analyse(DEFAULTS, cycles=1.0)
    return {
        "out": os.path.abspath(args.out), "webots": args.webots, "project": project, "world": world,
        "warmup_s": args.warmup_s, "measure_s": args.measure_s, "timeout": args.timeout, "keep_trials": args.keep_trials,
        # Limits. The original gait's joint envelope is what the hardware has already executed.
        "max_joint_speed": args.max_joint_speed, "max_torque": args.max_torque, "tilt_limit": args.tilt_limit,
        "range_margin": args.range_margin,
        "envelope_min": baseline["joint_min_deg"].tolist(), "envelope_max": baseline["joint_max_deg"].tolist(),
        # Score weights
        "w_lateral": args.w_lateral, "w_yaw": args.w_yaw, "w_tilt": args.w_tilt, "tilt_ok": args.tilt_ok,
    }


def print_table(rows):
    print(f"{'name':<22}{'score':>9}{'v_fwd mm/s':>12}{'v_lat mm/s':>12}{'yaw deg':>9}{'tilt max':>10}"
          f"{'torque p99':>12}{'joint deg/s':>13}  status")
    for name, o in rows:
        def g(key, scale=1.0):
            return o.get(key, float("nan")) * scale
        print(f"{name:<22}{o['score']:>9.4f}{g('v_forward', 1e3):>12.2f}{g('v_lateral', 1e3):>12.2f}"
              f"{g('yaw_change_deg'):>9.2f}{g('tilt_max_deg'):>10.1f}{g('torque_p99'):>12.2f}"
              f"{g('max_joint_speed_deg_s'):>13.0f}  {o['status']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--evaluate", nargs="+", metavar="PARAMS", help="score these parameter files ('defaults' = original)")
    ap.add_argument("--out", default=os.path.join(HERE, "tuning_runs", time.strftime("%Y%m%d_%H%M%S")))
    ap.add_argument("--webots", default=os.environ.get("WEBOTS", "webots"), help="Webots executable (or env WEBOTS)")
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--base", default=None, help="parameter file with the values that are not searched "
                    "(yaw gains, finish_stages...). Default: gait_params.json if it exists")
    ap.add_argument("--workers", type=int, default=3, help="Webots instances in parallel")
    ap.add_argument("--init-size", type=int, default=32, help="initial population (power of 2 works best)")
    ap.add_argument("--maxiter", type=int, default=15, help="generations")
    ap.add_argument("--minutes", type=float, default=None, help="stop the search after this wall time")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--top", type=int, default=5, help="candidates re-checked over a longer window at the end")
    ap.add_argument("--warmup-s", type=float, default=6)
    ap.add_argument("--measure-s", type=float, default=24)
    ap.add_argument("--timeout", type=float, default=300, help="seconds of wall time allowed per simulation")
    ap.add_argument("--keep-trials", action="store_true", help="keep the folder (log, result) of every simulation")
    g = ap.add_argument_group("limits")
    g.add_argument("--max-joint-speed", type=float, default=350.0, help="deg/s over 50 ms. The motor model of "
                   "MOSAC_Controller (8.4 V / KM) has a no-load speed of ~490 deg/s; 350 leaves room for load")
    g.add_argument("--max-torque", type=float, default=5.0, help="N.m, 99th percentile. The same motor model stalls at ~5.7")
    g.add_argument("--tilt-limit", type=float, default=20.0, help="deg, maximum body tilt")
    g.add_argument("--range-margin", type=float, default=10.0, help="deg a joint may go beyond the original gait's range")
    g = ap.add_argument_group("score weights")
    g.add_argument("--w-lateral", type=float, default=1.0)
    g.add_argument("--w-yaw", type=float, default=1.0, help="per rad/s of heading change")
    g.add_argument("--w-tilt", type=float, default=0.002, help="per deg of max tilt above --tilt-ok")
    g.add_argument("--tilt-ok", type=float, default=10.0)
    args = ap.parse_args()

    cfg = build_cfg(args)
    base = load_params(args.base) if args.base else load_params()

    if args.evaluate:
        rows = []
        for source in args.evaluate:
            params = load_params(source)
            rows.append((os.path.basename(source), evaluate(cfg, params)))
        print_table(rows)
        return

    objective = Objective(cfg, base)
    names = objective.names
    bounds = [SEARCH_SPACE[n] for n in names]
    lo, hi = np.array(bounds).T
    init = qmc.scale(qmc.Sobol(len(names), seed=args.seed).random(args.init_size), lo, hi)
    init[0] = np.clip([float(base[n]) for n in names], lo, hi)  # the current parameters are always a candidate
    started = time.time()
    best = {"score": -np.inf}

    def progress(intermediate_result):
        nonlocal best
        score = -intermediate_result.fun
        print(f"generation done  t={time.time() - started:5.0f} s  best score={score:.4f}  "
              f"{ {n: round(float(v), 2) for n, v in zip(names, intermediate_result.x)} }", flush=True)
        if args.minutes and time.time() - started > args.minutes * 60:
            print("time budget reached, stopping the search")
            raise StopIteration

    print(f"searching {names}\n  {args.init_size} candidates per generation, {args.workers} Webots in parallel, "
          f"results in {cfg['out']}", flush=True)
    result = differential_evolution(objective, bounds, init=init, maxiter=args.maxiter, mutation=(0.5, 1.0),
                                    recombination=0.8, seed=args.seed, tol=0, atol=0, polish=False,
                                    workers=args.workers, updating="deferred", callback=progress)

    # Re-check the best few over a window twice as long, so a gait that only looked good over a short one is dropped
    order = np.argsort(result.population_energies)[:args.top]
    print(f"re-checking the best {len(order)} candidates over {2 * args.measure_s:.0f} s", flush=True)
    finalists = []
    for i in order:
        params = objective.params(result.population[i])
        finalists.append((params, evaluate(cfg, params, measure_s=2 * args.measure_s)))
    finalists.sort(key=lambda pr: -pr[1]["score"])
    print_table([(f"finalist {k + 1}", o) for k, (p, o) in enumerate(finalists)])

    best_params, best_eval = finalists[0]
    with open(os.path.join(cfg["out"], "best_params.json"), "w") as f:
        json.dump({k: (round(v, 3) if isinstance(v, float) else v) for k, v in best_params.items()}, f, indent=2)
    with open(os.path.join(cfg["out"], "summary.json"), "w") as f:
        json.dump({"best_params": best_params, "best_eval": {k: v for k, v in best_eval.items() if k != "trace"},
                   "evaluations": int(result.nfev), "wall_s": round(time.time() - started)}, f, indent=2)
    print(f"{result.nfev} candidates simulated in {time.time() - started:.0f} s. Best: {cfg['out']}/best_params.json")


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
