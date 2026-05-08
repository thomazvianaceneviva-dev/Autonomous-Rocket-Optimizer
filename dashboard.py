"""
Dashboard
Run with: streamlit run dashboard.py
"""

import streamlit as st
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Agg")
import random
import sys
import os
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

from core.models import MotorDesign, EvalResult
from motor_sim.openmotor_sim import simulate_openmotor
from fligth.flight_simulator import simulate_flight

# ==========================================================
# PAGE CONFIG
# ==========================================================

st.set_page_config(
    page_title="Rocket Motor Optimizer",
    page_icon="🚀",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==========================================================
# STYLE
# ==========================================================

st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;400;600;700&display=swap');

html, body, [class*="css"] {
    font-family: 'Exo 2', sans-serif;
}

.stApp {
    background-color: #0a0c10;
    color: #c8d6e5;
}

h1, h2, h3 {
    font-family: 'Share Tech Mono', monospace !important;
    color: #f0a500 !important;
    letter-spacing: 0.05em;
}

.metric-card {
    background: #111520;
    border: 1px solid #1e2a3a;
    border-left: 3px solid #f0a500;
    border-radius: 4px;
    padding: 16px 20px;
    margin-bottom: 12px;
}

.metric-value {
    font-family: 'Share Tech Mono', monospace;
    font-size: 2rem;
    color: #f0a500;
    line-height: 1;
}

.metric-label {
    font-size: 0.75rem;
    color: #6b7f94;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    margin-top: 4px;
}

.status-ok  { color: #2ecc71; font-family: 'Share Tech Mono', monospace; }
.status-bad { color: #e74c3c; font-family: 'Share Tech Mono', monospace; }

.stButton > button {
    background: #f0a500;
    color: #0a0c10;
    font-family: 'Share Tech Mono', monospace;
    font-weight: 700;
    letter-spacing: 0.08em;
    border: none;
    border-radius: 2px;
    padding: 0.6rem 2rem;
    width: 100%;
    transition: all 0.2s;
}

.stButton > button:hover {
    background: #ffbe33;
    transform: translateY(-1px);
}

section[data-testid="stSidebar"] {
    background-color: #0d1117;
    border-right: 1px solid #1e2a3a;
}

.stSlider > div > div > div {
    background: #f0a500 !important;
}

hr {
    border-color: #1e2a3a;
}
</style>
""", unsafe_allow_html=True)

# ==========================================================
# SIDEBAR — PARAMETERS
# ==========================================================

st.sidebar.markdown("# ⚙ PARAMETERS")
st.sidebar.markdown("---")

st.sidebar.markdown("### Mission")
target_apogee = st.sidebar.number_input("Target Apogee (m)", value=1000, step=50)
total_mass    = st.sidebar.number_input("Total Rocket Mass (kg)", value=4.079, step=0.1, format="%.3f")
launch_angle  = st.sidebar.slider("Launch Angle (°)", 60, 90, 90)

st.sidebar.markdown("---")
st.sidebar.markdown("### Search Space")

st.sidebar.markdown("**Outer Diameter (mm)**")
od_min, od_max = st.sidebar.slider("", 30, 100, (50, 60), key="od")
st.sidebar.markdown("**Grain Length (mm)**")
gl_min, gl_max = st.sidebar.slider("", 30, 250, (60, 150), key="gl")
st.sidebar.markdown("**Core Diameter (mm)**")
cd_min, cd_max = st.sidebar.slider("", 5, 80, (15, 40), key="cd")
st.sidebar.markdown("**Throat Diameter (mm)**")
th_min, th_max = st.sidebar.slider("", 5, 40, (10, 20), key="th")

st.sidebar.markdown("---")
st.sidebar.markdown("### Optimizer")
n_random = st.sidebar.slider("Random Search Population", 10, 500, 100, step=10)
top_k    = st.sidebar.slider("Top K → CMA-ES", 5, 50, 20)

LOW  = np.array([od_min, gl_min, cd_min, th_min, 1], dtype=float)
HIGH = np.array([od_max, gl_max, cd_max, th_max, 4], dtype=float)

# ==========================================================
# HEADER
# ==========================================================

st.markdown("# 🚀 ROCKET MOTOR OPTIMIZER")
st.markdown("<p style='color:#6b7f94; font-family:Share Tech Mono; font-size:0.85rem'>Autonomous Geometric Optimization Pipeline — KNSB Propellant</p>", unsafe_allow_html=True)
st.markdown("---")

# ==========================================================
# TABS
# ==========================================================

tab1, tab2, tab3 = st.tabs(["🎯  OPTIMIZE", "🔬  TEST DESIGN", "📊  RESULTS"])

# ==========================================================
# TAB 1 — OPTIMIZE
# ==========================================================

with tab1:

    col_info, col_run = st.columns([2, 1])

    with col_info:
        st.markdown("### Mission Profile")
        c1, c2, c3 = st.columns(3)
        with c1:
            st.markdown(f"""
            <div class='metric-card'>
                <div class='metric-value'>{target_apogee}</div>
                <div class='metric-label'>Target Apogee (m)</div>
            </div>""", unsafe_allow_html=True)
        with c2:
            st.markdown(f"""
            <div class='metric-card'>
                <div class='metric-value'>{total_mass:.2f}</div>
                <div class='metric-label'>Total Mass (kg)</div>
            </div>""", unsafe_allow_html=True)
        with c3:
            st.markdown(f"""
            <div class='metric-card'>
                <div class='metric-value'>{launch_angle}°</div>
                <div class='metric-label'>Launch Angle</div>
            </div>""", unsafe_allow_html=True)

        st.markdown("### Pipeline")
        st.markdown(f"""
        <div class='metric-card'>
            <span style='color:#6b7f94; font-family:Share Tech Mono; font-size:0.8rem'>
            PHASE 1 &nbsp;→&nbsp; Random Search &nbsp;·&nbsp; {n_random} designs<br>
            PHASE 2 &nbsp;→&nbsp; CMA-ES &nbsp;·&nbsp; seeded with top {top_k} from phase 1
            </span>
        </div>""", unsafe_allow_html=True)

    with col_run:
        st.markdown("### Launch")
        run = st.button("▶  RUN OPTIMIZER")

    if run:
        results_dir = Path("./optimizer_workspace/results")
        results_dir.mkdir(parents=True, exist_ok=True)

        def random_design():
            return MotorDesign(
                outer_diam_mm   = random.uniform(LOW[0], HIGH[0]),
                grain_length_mm = random.uniform(LOW[1], HIGH[1]),
                core_diam_mm    = random.uniform(LOW[2], HIGH[2]),
                throat_mm       = random.uniform(LOW[3], HIGH[3]),
                grains          = random.randint(int(LOW[4]), int(HIGH[4])),
            )

        def evaluate_design(design):
            isp, burn, sim, prop_mass = simulate_openmotor(design)
            if sim is None or isp <= 0:
                return None, 0.0, 1e9
            try:
                apogee = simulate_flight(design, sim, prop_mass, total_mass, launch_angle)
                if isinstance(apogee, dict):
                    apogee = apogee["max_altitude"]
            except Exception:
                apogee = 0.0
            score = (apogee - target_apogee)**2 / target_apogee**2 - 0.005 * isp
            return isp, apogee, score

        # --- Phase 1 ---
        st.markdown("---")
        st.markdown("### Phase 1 — Random Search")
        progress = st.progress(0)
        status   = st.empty()
        rows     = []

        for i in range(n_random):
            d = random_design()
            isp, apogee, score = evaluate_design(d)
            row = asdict(d)
            row.update({"isp": isp or 0, "apogee_m": apogee, "score": score})
            rows.append(row)
            progress.progress((i + 1) / n_random)
            best = min(rows, key=lambda r: abs(r['apogee_m'] - target_apogee))
            status.markdown(f"<span class='status-ok'>Evaluated {i+1}/{n_random} — closest so far: {best['apogee_m']:.1f} m (Δ{best['apogee_m']-target_apogee:+.1f} m)</span>", unsafe_allow_html=True)

        df1 = pd.DataFrame(rows).sort_values("score")
        df1.to_csv(results_dir / "phase1_all.csv", index=False)
        df1.head(top_k).to_csv(results_dir / "phase1_topk.csv", index=False)

        status.markdown("<span class='status-ok'>✓ Phase 1 complete</span>", unsafe_allow_html=True)

        st.markdown("#### Top Results")
        st.dataframe(
            df1.head(10)[["outer_diam_mm","grain_length_mm","core_diam_mm","throat_mm","grains","isp","apogee_m","score"]].round(2),
            use_container_width=True
        )

        # --- Plot ---
        fig, axes = plt.subplots(1, 2, figsize=(12, 4), facecolor="#0a0c10")
        for ax in axes:
            ax.set_facecolor("#111520")
            ax.tick_params(colors="#6b7f94")
            for spine in ax.spines.values():
                spine.set_edgecolor("#1e2a3a")

        axes[0].scatter(df1["isp"], df1["apogee_m"], c="#f0a500", alpha=0.5, s=15)
        axes[0].axhline(target_apogee, color="#e74c3c", linestyle="--", linewidth=1, label=f"Target {target_apogee}m")
        axes[0].set_xlabel("ISP (s)", color="#6b7f94")
        axes[0].set_ylabel("Apogee (m)", color="#6b7f94")
        axes[0].set_title("ISP vs Apogee", color="#f0a500", fontsize=10)
        axes[0].legend(facecolor="#111520", edgecolor="#1e2a3a", labelcolor="#c8d6e5")

        axes[1].hist(df1["apogee_m"], bins=20, color="#f0a500", alpha=0.7, edgecolor="#0a0c10")
        axes[1].axvline(target_apogee, color="#e74c3c", linestyle="--", linewidth=1)
        axes[1].set_xlabel("Apogee (m)", color="#6b7f94")
        axes[1].set_ylabel("Count", color="#6b7f94")
        axes[1].set_title("Apogee Distribution", color="#f0a500", fontsize=10)

        plt.tight_layout()
        st.pyplot(fig)
        plt.close()

        st.session_state["phase1_df"] = df1
        best_row = df1.iloc[0]
        st.success(f"Phase 1 complete. Closest apogee: {best_row['apogee_m']:.1f} m (Δ{best_row['apogee_m']-target_apogee:+.1f} m) — Best ISP: {df1['isp'].max():.2f} s")


# ==========================================================
# TAB 2 — TEST DESIGN
# ==========================================================

with tab2:

    st.markdown("### Test a Specific Design")

    col_a, col_b = st.columns(2)
    with col_a:
        t_od = st.number_input("Outer Diameter (mm)", value=55.5, format="%.1f")
        t_gl = st.number_input("Grain Length (mm)", value=106.0, format="%.1f")
        t_cd = st.number_input("Core Diameter (mm)", value=29.0, format="%.1f")
    with col_b:
        t_th = st.number_input("Throat Diameter (mm)", value=16.0, format="%.1f")
        t_gr = st.number_input("Number of Grains", value=3, min_value=1, max_value=4)

    if st.button("▶  SIMULATE"):
        design = MotorDesign(
            outer_diam_mm   = t_od,
            grain_length_mm = t_gl,
            core_diam_mm    = t_cd,
            throat_mm       = t_th,
            grains          = int(t_gr),
        )

        with st.spinner("Running simulation..."):
            isp, burn, sim, prop_mass = simulate_openmotor(design)

        if sim is None or isp <= 0:
            st.error("Simulation failed — invalid design or pressure exceeded limits.")
        else:
            st.markdown("---")
            st.markdown("### Motor Results")
            c1, c2, c3, c4 = st.columns(4)
            metrics = [
                ("ISP", f"{isp:.2f} s"),
                ("Burn Time", f"{burn:.3f} s"),
                ("Prop Mass", f"{prop_mass:.4f} kg"),
                ("Avg Pressure", f"{sim.getAveragePressure()/1e5:.1f} bar"),
            ]
            for col, (label, val) in zip([c1, c2, c3, c4], metrics):
                with col:
                    st.markdown(f"""
                    <div class='metric-card'>
                        <div class='metric-value' style='font-size:1.4rem'>{val}</div>
                        <div class='metric-label'>{label}</div>
                    </div>""", unsafe_allow_html=True)

            c5, c6 = st.columns(2)
            with c5:
                st.markdown(f"""
                <div class='metric-card'>
                    <div class='metric-value' style='font-size:1.4rem'>{sim.getImpulse():.2f} Ns</div>
                    <div class='metric-label'>Total Impulse</div>
                </div>""", unsafe_allow_html=True)
            with c6:
                st.markdown(f"""
                <div class='metric-card'>
                    <div class='metric-value' style='font-size:1.4rem'>{sim.getMaxPressure()/1e5:.2f} bar</div>
                    <div class='metric-label'>Peak Pressure</div>
                </div>""", unsafe_allow_html=True)

            # Thrust curve
            try:
                thrust_data = np.array(sim.channels['Thrust'].getData())
                time_data   = np.array(sim.channels['Time'].getData())

                fig, ax = plt.subplots(figsize=(10, 3), facecolor="#0a0c10")
                ax.set_facecolor("#111520")
                ax.tick_params(colors="#6b7f94")
                for spine in ax.spines.values():
                    spine.set_edgecolor("#1e2a3a")
                ax.plot(time_data, thrust_data, color="#f0a500", linewidth=2)
                ax.fill_between(time_data, thrust_data, alpha=0.15, color="#f0a500")
                ax.set_xlabel("Time (s)", color="#6b7f94")
                ax.set_ylabel("Thrust (N)", color="#6b7f94")
                ax.set_title("Thrust Curve", color="#f0a500")
                plt.tight_layout()
                st.pyplot(fig)
                plt.close()
            except Exception:
                st.info("Thrust curve not available for this simulation.")

            # Flight
            st.markdown("### Flight Simulation")
            with st.spinner("Simulating trajectory..."):
                try:
                    result = simulate_flight(design, sim, prop_mass, total_mass, launch_angle)
                    if isinstance(result, dict):
                        apogee = result["max_altitude"]
                        rng    = result.get("range", 0)
                        sol    = result.get("solution")
                    else:
                        apogee = float(result)
                        sol    = None
                        rng    = 0

                    fa, fb = st.columns(2)
                    with fa:
                        st.markdown(f"""
                        <div class='metric-card'>
                            <div class='metric-value'>{apogee:.1f} m</div>
                            <div class='metric-label'>Max Altitude</div>
                        </div>""", unsafe_allow_html=True)
                    with fb:
                        delta = apogee - target_apogee
                        color = "#2ecc71" if abs(delta) < 50 else "#e74c3c"
                        st.markdown(f"""
                        <div class='metric-card'>
                            <div class='metric-value' style='color:{color}'>{delta:+.1f} m</div>
                            <div class='metric-label'>Δ from Target ({target_apogee} m)</div>
                        </div>""", unsafe_allow_html=True)

                    if sol is not None and hasattr(sol, 'y') and sol.y.shape[0] == 4:
                        fig2, ax2 = plt.subplots(figsize=(10, 3), facecolor="#0a0c10")
                        ax2.set_facecolor("#111520")
                        ax2.tick_params(colors="#6b7f94")
                        for spine in ax2.spines.values():
                            spine.set_edgecolor("#1e2a3a")
                        ax2.plot(sol.y[0], sol.y[1], color="#f0a500", linewidth=2)
                        ax2.set_xlabel("Range (m)", color="#6b7f94")
                        ax2.set_ylabel("Altitude (m)", color="#6b7f94")
                        ax2.set_title("Trajectory", color="#f0a500")
                        plt.tight_layout()
                        st.pyplot(fig2)
                        plt.close()

                except Exception as e:
                    st.error(f"Flight simulation error: {e}")


# ==========================================================
# TAB 3 — RESULTS
# ==========================================================

with tab3:
    st.markdown("### Saved Results")

    results_dir = Path("./optimizer_workspace/results")

    for fname in ["phase1_topk.csv", "phase2_cmaes.csv", "best_designs.csv", "phase1_all.csv"]:
        fpath = results_dir / fname
        if fpath.exists():
            df = pd.read_csv(fpath)
            st.markdown(f"#### {fname}")
            st.dataframe(df.round(3), use_container_width=True)

            csv = df.to_csv(index=False).encode()
            st.download_button(
                label=f"⬇ Download {fname}",
                data=csv,
                file_name=fname,
                mime="text/csv"
            )
            st.markdown("---")
        else:
            st.markdown(f"<span style='color:#6b7f94; font-family:Share Tech Mono; font-size:0.8rem'>{fname} — not found (run optimizer first)</span>", unsafe_allow_html=True)
