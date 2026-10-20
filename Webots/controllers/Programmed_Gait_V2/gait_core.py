"""Gait generator of Programmed_Gait_V2. Pure numpy (no Webots), so it can be tested and analysed offline.

It is the crawl gait of the original Programmed_Gait controller: 16 stages per cycle, one foot in the air at a time.
  lean    shorten the diagonal leg (and its neighbour) so the body tips away from the leg about to lift
  swing   lift one foot and move it forward along a parabola
  restore bring the leaning legs back to the standing height
  push    all four feet slide back under the body, which moves the body forward
Each leg is described by H (hip-to-foot height) and D1 (foot offset from the hip, bigger = further back),
both in mm, and inverse kinematics turns them into the femur and tibia servo angles.

Differences with the original: the stage sequence is a table, every constant comes from gait_config, and the feet can be
made to slide as for a turn (set_twist). With four feet planted, turning the body takes two things: a longer stride on
the outer side, and a sideways slide of the front feet against the rear feet. The sideways slide is done by the shoulder
(abduction) joints, which the original gait never moves.
"""
import math
from bisect import bisect_right

import numpy as np

NUM_STAGES = 16

BL, FL, BR, FR = 0, 1, 2, 3

# (kind, primary leg, secondary leg) for each stage. Lean/restore: the primary leg shortens by lean_primary and the
# secondary one by lean_secondary. Swing: the primary leg is the one in the air.
STAGES = [
    ("lean", BR, BL), ("swing", FL, None), ("restore", BR, BL), ("push", None, None),
    ("lean", FL, FR), ("swing", BR, None), ("restore", FL, FR), ("push", None, None),
    ("lean", BL, BR), ("swing", FR, None), ("restore", BL, BR), ("push", None, None),
    ("lean", FR, FL), ("swing", BL, None), ("restore", FR, FL), ("push", None, None),
]
PUSH_STAGES = tuple(i for i, s in enumerate(STAGES) if s[0] == "push")
# Each kind of stage takes a share of the cycle proportional to its weight in the parameters. The original gait has them all equal.
WEIGHT_KEYS = ("weight_lean", "weight_swing", "weight_restore", "weight_push")

# Leg geometry from the CAD (mm, rad)
L_FEMUR = 100.0
L_TIBIA = 100.70866
PSI = 2.51591 * np.pi / 180
# Tibia servo -> tibia linkage
A = 24.0
B = 40.5
C = 28.66176
D = 27.0
DELTA = 0.7984834029
EPSILON = 1.570796327
# Calibrated servo mid points
MID_POINT_SOLID_FEMUR = 42.65021  # deg
MID_POINT_SOLID_TIBIA = 16.75397157  # deg

# Position of each foot (BL, FL, BR, FR) about the middle of the support polygon, in mm, with the legs at mid stride.
# Measured in Webots at the standing pose from the positions of the foot solids (PBL, PFL, PBR, PFR).
FOOT_X = np.array([-125.0, 125.0, -125.0, 125.0])
FOOT_Y = np.array([101.8, 101.8, -101.8, -101.8])

# Shoulder (abduction) joints, in the order of the legs BL, FL, BR, FR. A positive angle moves the foot towards +y
# (to the left of the robot) on all four, which was checked in Webots.
SHOULDER_NAMES = ["BBL_joint", "BFL_joint", "BBR_joint", "BFR_joint"]

# Order of the 8 commanded femur and tibia joints: BL, FL, BR, FR
JOINT_NAMES = [
    "BL_servo_femur_joint", "BL_servo_tibia_joint",
    "FL_servo_femur_joint", "FL_servo_tibia_joint",
    "BR_servo_femur_joint", "BR_servo_tibia_joint",
    "FR_servo_femur_joint", "FR_servo_tibia_joint",
]


class GaitInfeasible(ValueError):
    """The requested foot position is out of the leg's reach."""


def servo_angles(H, D1):
    """Inverse kinematics. H and D1 are arrays with one value per leg (BL, FL, BR, FR), in mm.
    Returns the Webots joint targets in rad, ordered as JOINT_NAMES."""
    with np.errstate(invalid="ignore", divide="ignore"):
        # Femur and tibia angles of the two-link leg
        D2_sqrd = H**2 + D1**2
        D2 = np.sqrt(D2_sqrd)
        phi_femur = np.pi/2 - np.arctan(D1/H) - np.arccos((D2_sqrd + L_FEMUR**2 - L_TIBIA**2)/(2*L_FEMUR*D2))
        phi_tibia = np.pi/2 + np.arctan(D1/H) - np.arccos((D2_sqrd + L_TIBIA**2 - L_FEMUR**2)/(2*L_TIBIA*D2))
        phi_tibia = phi_tibia + PSI

        # Tibia angle -> tibia servo angle
        W_sqrd = C**2 + D**2 - 2*C*D*np.cos(np.pi + DELTA - EPSILON - phi_tibia)
        W = np.sqrt(W_sqrd)
        phi_tibia_servo = np.pi - DELTA - np.arccos((C**2 + W_sqrd - D**2)/(2*C*W)) - np.arccos((A**2 + W_sqrd - B**2)/(2*A*W))

    # Angles of the kinematics equations -> calibrated servo values
    phi_femur_servo = -phi_femur + MID_POINT_SOLID_FEMUR*np.pi/180
    phi_tibia_servo = -phi_tibia_servo + MID_POINT_SOLID_TIBIA*np.pi/180

    targets = np.empty(8)
    targets[0::2] = -phi_femur_servo
    targets[1::2] = -phi_tibia_servo
    if not np.all(np.isfinite(targets)):
        raise GaitInfeasible(f"foot out of reach: H={np.round(H, 1)} D1={np.round(D1, 1)}")
    return targets


class GaitGenerator:
    """Call step() once per control step.

    targets: the 8 femur/tibia joint targets (rad), ordered as JOINT_NAMES.
    shoulder_targets: the 4 shoulder joint targets (rad), ordered as SHOULDER_NAMES. They stay 0 unless set_twist() is
    called with a non zero turn or sideways stride, so the default gait is the original one.
    """

    def __init__(self, params, dt=0.01):
        self.p = params
        self.dt = dt
        self.stride_scale = np.ones(4)  # stride of each leg (BL, FL, BR, FR) as a fraction of parabola_length
        self.stride_y = np.zeros(4)     # sideways stride of each leg in mm per cycle, positive moves the body to +y
        self.t = 0.0
        self.stage = -1
        self.H = np.full(4, float(params["standing_height"]))
        # Stagger the legs so that each one is at the rear end of its stride when its swing stage comes
        self.D1 = -np.array([1.0, -2.0, -1.0, 0.0]) * params["parabola_length"] / 4
        self.lat = np.zeros(4)  # sideways offset of each foot in mm, positive towards +y
        self.swing_start = self.D1.copy()
        self.swing_start_lat = self.lat.copy()
        self.landing = np.zeros(4)      # where each foot lands (D1, sideways offset), fixed when its swing starts
        self.landing_lat = np.zeros(4)
        self.targets = servo_angles(self.H, self.D1)
        self.shoulder_targets = np.zeros(4)
        self._timing()

    def _timing(self):
        """Where each stage starts and how much of the cycle it takes, from the stage weights of the parameters."""
        self._weights = tuple(self.p[k] for k in WEIGHT_KEYS)
        weight = dict(zip(("lean", "swing", "restore", "push"), self._weights))
        w = np.array([weight[kind] for kind, _, _ in STAGES], dtype=float)
        self._width = w / w.sum()                           # fraction of the cycle
        self._edges = [0.0] + (np.cumsum(w) / w.sum())[:-1].tolist()  # where each stage starts

    def _locate(self, phase):
        """(stage, progress 0..1 in it) at this fraction of the cycle."""
        stage = min(bisect_right(self._edges, phase) - 1, NUM_STAGES - 1)
        return stage, (phase - self._edges[stage]) / self._width[stage]

    def stand(self):
        """Standing pose with every foot at the centre of its stride, which is where a turn on the spot leaves them. The
        walking start pose staggers the feet instead, so the first swings of a turn that starts from it are longer in one
        direction than in the other."""
        self.H[:] = self.p["standing_height"]
        self.D1[:] = 0.0
        self.lat[:] = 0.0
        self.swing_start = self.D1.copy()
        self.swing_start_lat = self.lat.copy()
        self.targets = servo_angles(self.H, self.D1)
        self.shoulder_targets = np.zeros(4)

    def use_params(self, params):
        """Carry on with other gait parameters (for instance a faster period and a lower foot lift for turning on the spot).
        Call it while the feet are centred and the stride is zero, at a stage boundary, which is where a stopped TurnController
        leaves the gait: the stage and the progress in it are kept, so the pose does not change. That only holds if the
        standing height and the lean are the same in both sets. The stage weights may differ."""
        for key in ("standing_height", "lean_primary", "lean_secondary"):
            if params[key] != self.p[key]:
                raise ValueError(f"cannot switch gait parameters: {key} differs ({self.p[key]} -> {params[key]})")
        stage, progress = self._locate(self.phase)
        self.p = params
        self._timing()
        self.t = (self._edges[stage] + progress * self._width[stage]) * params["gait_period"]

    def set_twist(self, omega, vy=0.0, forward=1.0):
        """Make the stance feet slide as for a body turning omega rad per cycle (positive = to the left) while moving
        vy mm per cycle sideways and forward * parabola_length mm per cycle forward (1 = the normal walk, 0 = on the spot,
        negative = backwards). Takes effect from the next push stage and from the next swing; the stage that is
        running keeps its landing point, so it can be called at any time."""
        L = self.p["parabola_length"]
        self.stride_scale[:] = forward - omega * FOOT_Y / L
        self.stride_y[:] = vy + omega * FOOT_X

    @property
    def phase(self):
        return (self.t % self.p["gait_period"]) / self.p["gait_period"]

    def step(self):
        p = self.p
        if self._weights != tuple(p[k] for k in WEIGHT_KEYS):
            self._timing()
        stage, prog = self._locate(self.phase)  # prog is 0..1 within the stage
        stage_time = p["gait_period"] * self._width[stage]
        if p["finish_stages"]:
            prog = min(1.0, prog + self.dt / stage_time)

        kind, leg, other = STAGES[stage]
        if stage != self.stage:
            self.stage = stage
            if kind == "swing":
                if p["finish_stages"]:
                    self.swing_start[leg] = self.D1[leg]
                else:  # original behaviour: every swing starts from the nominal rear end of the stride
                    self.swing_start[leg] = self.stride_scale[leg] * p["parabola_length"] / 2
                self.swing_start_lat[leg] = self.lat[leg]
                self.landing[leg] = -self.stride_scale[leg] * p["parabola_length"] / 2
                self.landing_lat[leg] = self.stride_y[leg] / 2

        standing = p["standing_height"]
        if kind == "lean":
            self.H[leg] = standing - p["lean_primary"] * prog
            self.H[other] = standing - p["lean_secondary"] * prog
        elif kind == "restore":
            self.H[leg] = standing - p["lean_primary"] * (1 - prog)
            self.H[other] = standing - p["lean_secondary"] * (1 - prog)
        elif kind == "swing":
            self.H[leg] = standing - p["parabola_height"] * (1 - 4*(0.5 - prog)**2)
            self.D1[leg] = self.swing_start[leg] + (self.landing[leg] - self.swing_start[leg]) * prog
            self.lat[leg] = self.swing_start_lat[leg] + (self.landing_lat[leg] - self.swing_start_lat[leg]) * prog
        else:  # push: every foot slides back by a quarter of its stride, and sideways by a quarter of its sideways stride
            share = self.dt / stage_time / 4
            self.D1 += self.stride_scale * p["parabola_length"] * share
            self.lat -= self.stride_y * share

        self.t += self.dt
        # A foot offset sideways by lat tilts the leg plane out by the shoulder angle; the leg keeps its height H
        self.shoulder_targets = np.arctan2(self.lat, self.H)
        self.targets = servo_angles(np.hypot(self.H, self.lat), self.D1)

    @property
    def push_time(self):
        """Seconds one push stage takes."""
        return self.p["gait_period"] * self._width[PUSH_STAGES[0]]

    def push_just_finished(self, previous_stage):
        """True on the first step after a push stage ended (previous_stage is the value of .stage before step())."""
        return previous_stage in PUSH_STAGES and self.stage != previous_stage


class YawController:
    """PI on the heading error. Call update() once per push stage (the only time the body moves).

    Returns the turn to command in rad per gait cycle (positive = to the left), to be passed to GaitGenerator.set_twist().
    Since a cycle has 4 push stages, kp = 4 would try to remove the whole error in one stage; much less is used.
    """

    def __init__(self, kp, ki, omega_max, ref):
        self.kp, self.ki, self.omega_max, self.ref = kp, ki, omega_max, ref
        self.integral = 0.0
        self.omega = 0.0

    def update(self, yaw):
        error = (self.ref - yaw + math.pi) % (2*math.pi) - math.pi
        self.integral += error
        if self.ki > 0:  # anti-windup: the integral term alone cannot exceed omega_max
            limit = self.omega_max / self.ki
            self.integral = max(-limit, min(limit, self.integral))
        self.omega = max(-self.omega_max, min(self.omega_max, self.kp*error + self.ki*self.integral))
        return self.omega


SPEED_WINDOW = 0.05  # s; a servo cannot follow single-step jumps, so speed is the movement over this window


def peak_joint_speed(targets_deg, dt, window=SPEED_WINDOW):
    """Fastest joint movement (deg/s) in a (steps, joints) array of joint targets, measured over a short window."""
    n = max(1, int(round(window / dt)))
    return float(np.abs(targets_deg[n:] - targets_deg[:-n]).max() / (n * dt))


def analyse(params, cycles=1.0, dt=0.01, omega=0.0):
    """Offline check of a parameter set: runs the gait for a number of cycles without any robot.

    omega is a constant turn in rad per cycle (see GaitGenerator.set_twist). Returns a dict with the commanded joint range
    (deg) and the fastest joint movement (deg/s). Raises GaitInfeasible if a foot goes out of reach.
    """
    gen = GaitGenerator(params, dt)
    gen.set_twist(omega)
    steps = int(round(cycles * params["gait_period"] / dt))
    traj = np.empty((steps + 1, 8))
    traj[0] = gen.targets
    for i in range(steps):
        gen.step()
        traj[i + 1] = gen.targets
    deg = np.degrees(traj)
    return {
        "joint_min_deg": deg.min(axis=0),
        "joint_max_deg": deg.max(axis=0),
        "max_joint_speed_deg_s": peak_joint_speed(deg, dt),
    }
