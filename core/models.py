from dataclasses import dataclass


@dataclass
class MotorDesign:
    outer_diam_mm: float
    grain_length_mm: float
    core_diam_mm: float
    throat_mm: float
    grains: int


@dataclass
class EvalResult:
    isp: float
    apogee_m: float
    burn_time_s: float
    score: float