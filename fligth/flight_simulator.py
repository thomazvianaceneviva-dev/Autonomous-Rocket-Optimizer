import numpy as np
from scipy.integrate import solve_ivp
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from core.models import MotorDesign

CD = 0.85
G = 9.80665
R_AIR = 287.05
T0 = 288.15
P0 = 101325
L = 0.0065

def air_density(h: float):
    h = max(0.0, h)
    T = T0 - L * h
    p = P0 * (T / T0) ** (G / (L * R_AIR))
    return p / (R_AIR * T)

def build_thrust_func(sim):
    try:
        thrust = np.array(sim.channels['Thrust'].getData())
        time = np.array(sim.channels['Time'].getData())
        return lambda t: float(np.interp(t, time, thrust, left=0.0, right=0.0))
    except Exception:
        try:
            I = float(sim.getImpulse())
            tb = float(sim.getBurnTime())
            F = I / tb if tb > 0 else 0.0
            return lambda t: F if t <= tb else 0.0
        except Exception:
            return lambda t: 0.0

def simulate_flight(design: MotorDesign, sim, prop_mass: float, total_mass: float, launch_angle_deg: float = 80.0):
    burn_time = float(sim.getBurnTime())
    thrust = build_thrust_func(sim)
    dry_mass = total_mass - prop_mass
    A_ref = np.pi * (design.outer_diam_mm / 2000.0) ** 2
    theta = np.radians(launch_angle_deg)

    def mass(t):
        if t >= burn_time:
            return dry_mass
        return dry_mass + prop_mass * (1 - t / burn_time)

    def eq(t, y):
        x, h, vx, vy = y
        rho = air_density(h)
        m = mass(t)
        v = np.sqrt(vx**2 + vy**2) + 1e-9
        # arrasto (vetorial)
        Fd = 0.5 * rho * CD * A_ref * v**2
        ax_d = -Fd * (vx / v) / m
        ay_d = -Fd * (vy / v) / m
        # gravidade
        ay_g = -G
        # thrust (assumido alinhado com eixo inicial do foguete)
        Ft = thrust(t)
        ax_t = (Ft / m) * np.cos(theta)
        ay_t = (Ft / m) * np.sin(theta)
        ax = ax_d + ax_t
        ay = ay_d + ay_t + ay_g
        return [vx, vy, ax, ay]

    def ground_hit(t, y):
        return y[1] - 0.1

    ground_hit.terminal = True
    ground_hit.direction = -1

    sol = solve_ivp(
        eq,
        (0, 300),
        [0.0, 0.0, 0.0, 0.0],
        events=[ground_hit],
        max_step=0.05,
        rtol=1e-6,
        atol=1e-8
    )

    return float(np.max(sol.y[1]))
