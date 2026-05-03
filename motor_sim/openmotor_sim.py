import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from motorlib.motor import Motor
from motorlib.nozzle import Nozzle, eRatioFromPRatio
from motorlib.propellant import Propellant, PropellantTab
from motorlib.grains.bates import BatesGrain
from core.models import MotorDesign


MAX_PRES   = 4e6
PROP_GAMMA = 1.13
AMB_PRES   = 101325.0
NOZZLE_EFFICIENCY = 0.832 # Efficiency here should be lower to adjust ISP; appears to be a difference in values for the "getajustedthrustcoeff" function which may be using higher efficiency than in the app

# ==========================================================
# HELPERS
# ==========================================================

def _expansion(p: float) -> float:
    """
    Calculate the optimal expansion ratio (Aexit/Athroat)
    for the simulated chamber pressure.
    """
    try:
        ratio = AMB_PRES / p
        exp   = 1.0 / eRatioFromPRatio(PROP_GAMMA, ratio)
        return float(np.clip(exp, 1.1, 8.0))
    except Exception:
        return 4.0  # conservative fallback


def _nozzle(throat: float, exp: float) -> Nozzle:
    n = Nozzle()
    n.setProperty("throat",       throat)
    n.setProperty("exit",         throat * np.sqrt(exp))
    n.setProperty("efficiency",   NOZZLE_EFFICIENCY)
    n.setProperty("divAngle",     15.0)
    n.setProperty("convAngle",    45.0)
    n.setProperty("throatLength", 0.005)
    n.setProperty("slagCoeff",    0.0)
    n.setProperty("erosionCoeff", 0.0)
    return n


def _prop() -> Propellant: # All values obtained from KNSB Nakka OpenMotor
    p = Propellant()
    p.setProperty("name",    "KNSB")
    p.setProperty("density", 1749.7)  # kg/m³ — 1.7497 g/cm³ of your propellant

    t = PropellantTab()
    t.setProperty("minPressure", 7.03e6)
    t.setProperty("maxPressure", 1.067e7)
    t.setProperty("a",           4.18e-3)
    t.setProperty("n",           0.059)
    t.setProperty("k",           PROP_GAMMA)
    t.setProperty("t",           1520.0)
    t.setProperty("m",           39.9)   # g/mol

    p.addTab(t)
    return p


def _grains(design: MotorDesign) -> list:
    grains = []
    n = max(1, int(round(design.grains)))
    for _ in range(n):
        g = BatesGrain()
        g.setProperty("diameter",     design.outer_diam_mm  / 1000.0)
        g.setProperty("length",       design.grain_length_mm / 1000.0)
        g.setProperty("coreDiameter", design.core_diam_mm   / 1000.0)
        grains.append(g)
    return grains


# ==========================================================
# MAIN SIMULATION — 2 PASSES
# ==========================================================

def simulate_openmotor(design: MotorDesign):
    """
    Pass 1: simulates with neutral expansion (4.0) to obtain actual chamber pressure.
    Pass 2: simulates with optimal expansion calculated from the actual pressure.

    Returns:
        isp (s), burn_time (s), sim2 (SimulationResult), prop_mass (kg)
    Returns 0.0, 0.0, None, 0.0 in case of invalid design or error.
    """

    # --------------------------------------------------
    # GEOMETRIC FILTERS
    # --------------------------------------------------
    if design.core_diam_mm >= design.outer_diam_mm:
        return 0.0, 0.0, None, 0.0

    if design.throat_mm <= 0:
        return 0.0, 0.0, None, 0.0

    throat = design.throat_mm / 1000.0

    try:
        # ==================================================
        # PASS 1 — provisional expansion, obtains actual pressure
        # ==================================================
        m1            = Motor()
        m1.config.setProperty("ambPressure", AMB_PRES)  # <-- debug: makes the motor operate at atmospheric pressure
        m1.nozzle     = _nozzle(throat, 4.0)
        m1.propellant = _prop()
        m1.grains     = _grains(design)

        s1 = m1.runSimulation()

        if not s1.success:
            return 0.0, 0.0, None, 0.0

        try:
            if s1.getMaxPressure() > MAX_PRES:
                return 0.0, 0.0, None, 0.0
        except Exception:
            pass

        try:
            p_real = s1.getAveragePressure()
            if not np.isfinite(p_real) or p_real <= 0:
                p_real = 2.5e6
        except Exception:
            p_real = 2.5e6

        # ==================================================
        # OPTIMAL EXPANSION with actual pressure
        # ==================================================
        exp = _expansion(p_real)

        # Temporary debug
        exit_diam_mm = (throat * np.sqrt(exp)) * 1000
        print(f"[DEBUG] Calculated expansion: {exp:.4f}")
        print(f"[DEBUG] Exit diameter:     {exit_diam_mm:.2f} mm")
        print(f"[DEBUG] Chamber pressure:     {p_real/1e5:.2f} bar")

        # ==================================================
        # PASS 2 — optimal nozzle, final result
        # ==================================================
        m2            = Motor()
        m2.config.setProperty("ambPressure", AMB_PRES)  # <-- debug: makes the motor operate at atmospheric pressure
        m2.nozzle     = _nozzle(throat, exp)
        m2.propellant = _prop()
        m2.grains     = _grains(design)

        s2 = m2.runSimulation()

        if not s2.success:
            return 0.0, 0.0, None, 0.0

        try:
            if s2.getMaxPressure() > MAX_PRES:
                return 0.0, 0.0, None, 0.0
        except Exception:
            pass

        # error alert filter
        erros = [a for a in s2.alerts if a.level.name == "ERROR"]
        if erros:
            return 0.0, 0.0, None, 0.0

        # --------------------------------------------------
        # RESULTS
        # --------------------------------------------------
        # AFTER (fixed) — Direct physical ISP
        G0 = 9.80665
        try:
            impulse   = float(s2.getImpulse())
            prop_mass = float(s2.getPropellantMass())
            isp = (impulse / (prop_mass * G0)) if prop_mass> 0 else 0.0 
        except Exception:
            isp = 0.0

        # Diagnostic — remove after confirming
        try:
            print(f"[DEBUG] Impulse:       {s2.getImpulse():.2f} Ns")
            print(f"[DEBUG] Prop mass:     {s2.getPropellantMass():.4f} kg")
            print(f"[DEBUG] ISP final:     {isp:.2f} s")
            print(f"[DEBUG] Average pressure: {s2.getAveragePressure()/1e5:.1f} bar")
        except Exception:
            pass

        try:
            burn = float(s2.getBurnTime())
        except Exception:
            burn = 0.0

        try:
            prop_mass = float(s2.getPropellantMass())
        except Exception:
            prop_mass = 0.0

        if not np.isfinite(isp)  or isp  < 0: isp  = 0.0
        if not np.isfinite(burn) or burn < 0: burn = 0.0

        return float(isp), float(burn), s2, float(prop_mass)

    except Exception as e:
        import traceback
        traceback.print_exc()
        return 0.0, 0.0, None, 0.0