"""
config.py -- Default paths, solver presets, and ALNS parameter definitions.
"""

import os

# ── Paths (relative to the project root, i.e., the parent of ./python) ────────
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

SOLVER_BIN      = os.path.join(PROJECT_ROOT, "build", "cpp", "core", "darpmd_run")
GENERATOR_MAIN  = os.path.join(PROJECT_ROOT, "python", "mddarp-gen", "main.py")
VIEWER_MAIN     = os.path.join(PROJECT_ROOT, "python", "mddarp-res-viewer", "main.py")

TMP_DIR         = os.path.join(os.path.dirname(__file__), "tmp")


# ── Solver method choices ──────────────────────────────────────────────────────
METHODS = ["ILP", "ILPSoft", "ALNS", "ALNS_SP", "ALNS_SC"]

METHOD_DESCRIPTIONS = {
    "ILP":     "Integer Linear Program -- exact solver via CPLEX",
    "ILPSoft": "ILP with soft constraints -- allows minor infeasibilities",
    "ALNS":    "Adaptive Large Neighbourhood Search -- fast metaheuristic",
    "ALNS_SP": "ALNS + Set Partitioning post-processing",
    "ALNS_SC": "ALNS + Set Covering post-processing",
}


# ── Built-in solver presets ────────────────────────────────────────────────────
# Each preset exposes only the most relevant knobs; advanced params stay hidden
# unless the user opens the "Advanced" panel.

PRESETS = {
    "Quick ILP (small instances)": {
        "method": "ILP",
        "time_limit": 60,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "Exact ILP solve with a 60-second time cap. Good for ≤ 8 requests.",
    },
    "ILP -- No time limit": {
        "method": "ILP",
        "time_limit": None,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "Exact ILP without any time limit. Use for small/medium instances.",
    },
    "ILP Soft (relaxed)": {
        "method": "ILPSoft",
        "time_limit": None,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "Soft-constraint ILP: allows small violations, finds solutions faster.",
    },
    "ALNS -- Fast (2 min)": {
        "method": "ALNS",
        "time_limit": 120,
        "seed": 42,
        "verbose": True,
        "enable_gice": True,
        "enable_nr": True,
        "description": "Pure ALNS metaheuristic with 2-minute budget. Great for large instances.",
    },
    "ALNS -- Balanced (5 min)": {
        "method": "ALNS",
        "time_limit": 300,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": True,
        "description": "ALNS with NR enabled for improved solution quality in 5 minutes.",
    },
    "ALNS_SP -- High quality (10 min)": {
        "method": "ALNS_SP",
        "time_limit": 600,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "ALNS followed by Set-Partitioning refinement. Best quality / time trade-off.",
    },
    "ALNS_SC -- High quality (10 min)": {
        "method": "ALNS_SC",
        "time_limit": 600,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "ALNS followed by Set-Covering refinement. Allows route overlap in post-step.",
    },
    "Custom": {
        "method": "ALNS",
        "time_limit": 300,
        "seed": 42,
        "verbose": True,
        "enable_gice": False,
        "enable_nr": False,
        "description": "All parameters are yours to set.",
    },
}

DEFAULT_PRESET = "ALNS -- Balanced (5 min)"


# ── ALNS parameter schema (order matters -- passed positionally via --alnsParams) ─
# Each entry: (key, label, default_value, tooltip)
ALNS_PARAM_SCHEMA = [
    ("max_iter",        "Max Iterations",           10000, "Maximum number of ALNS iterations"),
    ("segment_size",    "Segment Size",              100,   "Iterations per weight-update segment"),
    ("sigma1",          "σ₁ (best global reward)",  33,    "Score awarded when a new global best is found"),
    ("sigma2",          "σ₂ (better reward)",       13,    "Score awarded when solution improves current"),
    ("sigma3",          "σ₃ (accepted reward)",     9,     "Score awarded for an accepted worse solution"),
    ("r_decay",         "Reaction Factor (r)",      18,    "Weight decay factor (integer × 0.01 internally)"),
    ("start_temp",      "Start Temperature",        500,   "Initial SA temperature (0 = auto)"),
    ("cool_rate",       "Cooling Rate",             99,    "Cooling rate (integer × 0.01 internally)"),
    ("noise_factor",    "Noise Factor",             10,    "Noise strength in greedy insertion (× 0.01)"),
]
