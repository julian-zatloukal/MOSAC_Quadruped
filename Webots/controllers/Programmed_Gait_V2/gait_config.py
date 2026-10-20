"""Parameters of the Programmed_Gait_V2 controller, shared by the controller and the tuner.

DEFAULTS reproduce the original Programmed_Gait controller. The tuned values live in gait_params.json
(next to this file) and are merged over the defaults when the controller starts.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = os.path.join(HERE, "gait_params.json")

DEFAULTS = {
    # Gait shape
    "gait_period": 16.0,       # s, duration of one full 16-stage cycle
    "standing_height": 160.0,  # mm, hip-to-foot distance of a leg that is on the floor
    "parabola_height": 40.0,   # mm, how high a swinging foot is lifted
    "parabola_length": 100.0,  # mm, stride length
    "lean_primary": 20.0,      # mm, how much the diagonal leg shortens before a lift (shifts the weight)
    "lean_secondary": 10.0,    # mm, same for its neighbour
    # False = the original sampling: the last control step of every stage stops one step short of the stage's end
    # point, so a swung foot hovers ~6 mm above the floor through the next push stage and then drops in one step.
    # True = each step aims at where the foot should be at the END of the step, so every stage completes.
    "finish_stages": False,
    # How the cycle is shared out between the kinds of stage: each takes a part proportional to its weight. Only the ratios
    # matter. The swing is where the joints move fastest, so giving it more of the cycle than the others lets the whole cycle
    # run faster without any joint moving faster.
    "weight_lean": 1.0,
    "weight_swing": 1.0,
    "weight_restore": 1.0,
    "weight_push": 1.0,
    # Start-up
    "init_delay": 2.0,         # s standing still before walking
    # Yaw feedback: PI on the heading error, updated once per push stage (4 per cycle). It makes the stance feet slide
    # as for a turn: longer stride on the outer side, and the shoulder joints slide the front feet against the rear ones.
    # Disabled (and the shoulders left alone, as in the original) while yaw_kp and yaw_ki are 0.
    "yaw_kp": 0.0,             # rad turned per cycle, per rad of heading error
    "yaw_ki": 0.0,             # same, per rad of heading error accumulated over the push stages
    "yaw_max": 0.2,            # largest turn commanded, rad per cycle
    "yaw_ref_deg": 0.0,        # heading to hold, deg, in the world frame (0 = walking along +x)
}

# Parameters the tuner searches: name -> (low, high). Everything else keeps its value from gait_params.json.
SEARCH_SPACE = {
    "gait_period": (2.0, 12.0),
    "standing_height": (145.0, 175.0),
    "parabola_height": (20.0, 60.0),
    "parabola_length": (60.0, 150.0),
    "lean_primary": (0.0, 40.0),
    "lean_secondary": (0.0, 25.0),
    "finish_stages": (0.0, 1.0),  # a switch: on from 0.5
}

# More parameters tune_gait.py can search when asked to with --search (these are not in the default search)
EXTRA_SEARCH_SPACE = {
    "weight_lean": (0.2, 3.0),
    "weight_swing": (0.2, 3.0),
    "weight_restore": (0.2, 3.0),
    "weight_push": (0.2, 3.0),
}


def load_params(source=None):
    """DEFAULTS, overridden by a JSON file.

    source: path to a JSON file, "defaults" for the original constants only, or None to use the GAIT_PARAMS
    environment variable and then gait_params.json (if it exists).
    """
    if source is None:
        source = os.environ.get("GAIT_PARAMS")
    if source is None and os.path.exists(PARAMS_FILE):
        source = PARAMS_FILE

    params = dict(DEFAULTS)
    if source and source != "defaults":
        with open(source) as f:
            overrides = json.load(f)
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError(f"Unknown gait parameter(s) in {source}: {sorted(unknown)}")
        params.update(overrides)
    return params
