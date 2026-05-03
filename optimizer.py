"""
rocket_motor_optimizer.py

PIPELINE:
1) Random Search
2) OpenMotor simulation (2-pass)
3) Flight simulation (ODE)
4) CMA-ES refinement
"""

import uuid
import random
from pathlib import Path
from dataclasses import asdict

import numpy as np
import pandas as pd
import cma

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from core.models import MotorDesign, EvalResult
from motor_sim.openmotor_sim import simulate_openmotor
from fligth.flight_simulator import simulate_flight


# ==========================================================
# CONFIG
# ==========================================================

WORKDIR = Path("./optimizer_workspace")
RESULTS_DIR = WORKDIR / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

TARGET_APOGEE = 1000.0
TOTAL_MASS = 4.079

CMA_SIGMA0 = 0.25
CMA_MAX_ITER = 5
CMA_POPSIZE = 24

LOW = np.array([50, 60, 15, 10, 1], dtype=float)
HIGH = np.array([60, 150, 40, 20, 4], dtype=float)


# ==========================================================
# USER INPUT (fixed)
# ==========================================================

N_RANDOM = int(input("Phase 1 population (random search): "))
TOP_K = int(input("Top K for CMA-ES: "))


# ==========================================================
# DESIGN <-> VECTOR
# ==========================================================

def design_to_vector(d: MotorDesign):
    arr = np.array([
        d.outer_diam_mm,
        d.grain_length_mm,
        d.core_diam_mm,
        d.throat_mm,
        d.grains,
    ], dtype=float)

    return (arr - LOW) / (HIGH - LOW)


def vector_to_design(x):
    x = np.clip(x, 0, 1)
    vals = LOW + x * (HIGH - LOW)

    return MotorDesign(
        outer_diam_mm=float(vals[0]),
        grain_length_mm=float(vals[1]),
        core_diam_mm=float(vals[2]),
        throat_mm=float(vals[3]),
        grains=max(1, int(round(vals[4]))),
    )


# ==========================================================
# RANDOM DESIGN
# ==========================================================

def random_design():
    return MotorDesign(
        outer_diam_mm=random.uniform(LOW[0], HIGH[0]),
        grain_length_mm=random.uniform(LOW[1], HIGH[1]),
        core_diam_mm=random.uniform(LOW[2], HIGH[2]),
        throat_mm=random.uniform(LOW[3], HIGH[3]),
        grains=random.randint(int(LOW[4]), int(HIGH[4])),
    )


# ==========================================================
# OBJECTIVE FUNCTION
# ==========================================================

def evaluate_design(design: MotorDesign) -> EvalResult:

    isp, burn, sim, prop_mass = simulate_openmotor(design)
    print(f"DEBUG: isp={isp}, burn={burn}, sim={sim}, prop_mass={prop_mass}")

    # safe fallback
    if sim is None or isp <= 0:
        return EvalResult(
            isp=0.0,
            apogee_m=0.0,
            burn_time_s=0.0,
            score=1e9
        )

    try:
        apogee = simulate_flight(
            design,
            sim,
            prop_mass,
            TOTAL_MASS
        )
    except Exception:
        apogee = 0.0

    # STABILIZED SCORE (important for CMA-ES)
    score = (
        (apogee - TARGET_APOGEE) ** 2 / (TARGET_APOGEE ** 2)
        - 0.005 * isp
    )

    return EvalResult(
        isp=isp,
        apogee_m=apogee,
        burn_time_s=burn,
        score=score
    )


# ==========================================================
# PHASE 1 - RANDOM SEARCH
# ==========================================================

def phase1_random_search():

    print(f"\nPHASE 1 - Random Search ({N_RANDOM})")

    rows = []

    for i in range(N_RANDOM):

        d = random_design()
        r = evaluate_design(d)

        row = asdict(d)
        row.update(asdict(r))
        rows.append(row)

        if i % max(1, N_RANDOM // 10) == 0:
            print(f"{i}/{N_RANDOM}")

    df = pd.DataFrame(rows)
    df = df.sort_values("score")

    df.to_csv(RESULTS_DIR / "phase1_all.csv", index=False)

    top = df.head(TOP_K)
    top.to_csv(RESULTS_DIR / "phase1_topk.csv", index=False)

    return top


# ==========================================================
# PHASE 2 - CMA-ES
# ==========================================================

def phase2_cmaes(top_df):

    print("\nPHASE 2 - CMA-ES")

    designs = []

    for _, row in top_df.iterrows():
        d = MotorDesign(
            row.outer_diam_mm,
            row.grain_length_mm,
            row.core_diam_mm,
            row.throat_mm,
            int(row.grains),
        )
        designs.append(design_to_vector(d))

    mu0 = np.mean(np.array(designs), axis=0)

    es = cma.CMAEvolutionStrategy(
        mu0,
        CMA_SIGMA0,
        {
            "popsize": CMA_POPSIZE,
            "maxiter": CMA_MAX_ITER,
            "bounds": [0, 1],
        }
    )

    history = []

    while not es.stop():

        xs = es.ask()

        fitness = []
        rows = []

        for x in xs:

            d = vector_to_design(x)
            r = evaluate_design(d)

            fitness.append(r.score)

            row = asdict(d)
            row.update(asdict(r))
            rows.append(row)

        es.tell(xs, fitness)

        best_idx = int(np.argmin(fitness))
        best = rows[best_idx]

        history.append(best)

        print(
            f"gen={len(history):02d} "
            f"score={best['score']:.4f} "
            f"apo={best['apogee_m']:.1f} "
            f"isp={best['isp']:.2f}"
        )

        if es.stop():
            break

    df = pd.DataFrame(history)
    df = df.sort_values("score")

    df.to_csv(RESULTS_DIR / "phase2_cmaes.csv", index=False)

    return df


# ==========================================================
# MAIN
# ==========================================================

def main():

    print("Rocket Motor Optimizer")

    top = phase1_random_search()
    final = phase2_cmaes(top)

    print("\nTOP RESULTS:")
    print(final.head(10))

    final.head(10).to_csv(
        RESULTS_DIR / "best_designs.csv",
        index=False
    )


if __name__ == "__main__":
    main()

# Temporarily added at the end for debugging the simulation

# from motor_sim.openmotor_sim import simulate_openmotor
# from core.models import MotorDesign

# test = MotorDesign(
#     outer_diam_mm   = 55.5,
#     grain_length_mm = 106.0,
#     core_diam_mm    = 29.0,
#     throat_mm       = 16.0,
#     grains          = 3,
# )

# isp, burn, sim, prop_mass = simulate_openmotor(test)

# print(f"ISP:        {isp:.2f} s")
# print(f"Burn time:  {burn:.3f} s")
# print(f"Prop mass:  {prop_mass:.4f} kg")

# try:
#     print(f"Max pres:   {sim.getMaxPressure()/1e5:.2f} bar")
#     print(f"Avg pres:   {sim.getAveragePressure()/1e5:.2f} bar")
#     print(f"Impulse:    {sim.getImpulse():.2f} Ns")
#     print(f"Avg thrust: {sim.getAverageForce():.2f} N")
# except Exception as e:
#     print(f"Error: {e}")