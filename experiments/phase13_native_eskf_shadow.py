"""Native MuJoCo shadow-filter comparison: no estimator-to-control feedback.

Two development routes (one previously safe, one known collision failure).
The first is re-run without shadow to establish baseline determinism.
Do not classify a simulated flight as safe on the basis of ESKF metrics alone.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

SCENARIOS=((91311,"shortest"),(48221,"rf_aware"))

def run_study(output="results/phase13_native_eskf_shadow"):
    dest=Path(output);dest.mkdir(parents=True,exist_ok=True)
    rows=[]
    for seed,method in SCENARIOS:
        solids=INDUSTRIAL+tuple(city(seed))
        points=staged_route(method,solids,transit_z=10.,localization_sigma_m=.1)
        ref=quintic_reference(points,solids,radius_m=.32,sigma_m=.1,
            clearance_margin_m=.55,max_speed_m_s=1.,
            max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        for shadow in ((False,True) if seed==91311 else (True,)):
            flight=run("heavy",duration=240,wind=(.25 if seed==91311 else .35),
                city_seed=seed,wind_phase=(seed%19)*.12,
                route=points,trajectory_reference=ref,
                out=str(dest/f"flight_{seed}_{method}_{'shadow' if shadow else 'legacy'}.csv"),
                controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
                dynamic_replanning=False,eskf_shadow=shadow,
                localization_drift_m_sqrt_s=.03,
                localization_noise_sigma_m=.02,localization_seed=seed+71,
                lidar_blackout_window=(45.,75.))
            x={"seed":seed,"route":method,"shadow":shadow,
               "safe":bool(flight["completed"] and flight["contact_samples"]==0
                           and flight["min_clearance_m"]>=.25),
               "completed":flight["completed"],"elapsed_s":flight["elapsed_s"],
               "contact_samples":flight["contact_samples"],
               "min_clearance_m":flight["min_clearance_m"],
               "legacy_ate_rmse_m":flight["navigation_ate_rmse_m"],
               "legacy_max_error_m":flight["navigation_max_error_m"],
               "eskf_ate_rmse_m":flight["eskf_shadow_ate_rmse_m"],
               "eskf_max_error_m":flight["eskf_shadow_max_error_m"],
               "eskf_coverage99":flight["eskf_shadow_99pct_coverage"],
               "eskf_scan_accepted":flight["eskf_shadow_scan_accepted"],
               "eskf_scan_rejected":flight["eskf_shadow_scan_rejected"]}
            print("NATIVE",json.dumps(x),flush=True)
            rows.append(x)
    control=rows[0];observed=rows[1]
    unchanged=(control["completed"]==observed["completed"] and
               control["contact_samples"]==observed["contact_samples"] and
               control["min_clearance_m"]==observed["min_clearance_m"] and
               control["elapsed_s"]==observed["elapsed_s"] and
               control["legacy_ate_rmse_m"]==observed["legacy_ate_rmse_m"])
    summary={"scope":"native MuJoCo, shared *synthetic* sensor stream, shadow ESKF only",
             "no_shadow_changes_baseline":unchanged,"cases":rows,
             "warnings":["Prior map known exactly; observed LiDAR rays are synthetic",
                         "Shadow filter not driving flight controller or safety supervisor",
                         "Covariance empirical coverage is computed against simulator truth for evaluation only",
                         "Two development scenarios only, not independent heldout"]}
    (dest/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    if not unchanged:raise AssertionError("Shadow monitor changed baseline closed-loop outcomes")
    return summary

if __name__=="__main__":run_study()
