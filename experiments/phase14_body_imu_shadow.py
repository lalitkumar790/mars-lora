"""Phase 14: native body-frame 15-state ESKF in shadow, paired known cases.

The only signal connected to the flight controller is the *unchanged* legacy
hybrid estimator. The new 15-state estimator sees independently simulated
body-frame specific force and angular rates derived from MuJoCo truth only
in the sensor generator, with separate RNG. All 3D/planar range observations
are reused from the legacy run. No safety claim from shadow error alone.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

CASES=((91311,"shortest",.25),(48221,"rf_aware",.35))

def evaluate(out="results/phase14_body_imu_shadow"):
    dest=Path(out)
    dest.mkdir(parents=True,exist_ok=True)
    rows=[]
    for seed,method,wind in CASES:
        solids=INDUSTRIAL+tuple(city(seed))
        route=staged_route(method,solids,transit_z=10.,localization_sigma_m=.1)
        reference=quintic_reference(route,solids,radius_m=.32,sigma_m=.1,
            clearance_margin_m=.55,max_speed_m_s=1.,
            max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        for enabled in ((False,True) if seed==91311 else (True,)):
            flight=run("heavy",duration=240,wind=wind,city_seed=seed,
                wind_phase=(seed%19)*.12,route=route,trajectory_reference=reference,
                out=str(dest/f"flight_{seed}_{method}_{'15state' if enabled else 'baseline'}.csv"),
                controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
                dynamic_replanning=False,eskf15_shadow=enabled,
                localization_drift_m_sqrt_s=.03,localization_noise_sigma_m=.02,
                localization_seed=seed+71,lidar_blackout_window=(45.,75.))
            record={
                "seed":seed,"method":method,"shadow":enabled,
                "completed":flight["completed"],
                "safe":bool(flight["completed"] and flight["contact_samples"]==0
                            and flight["min_clearance_m"]>=.25),
                "contact_samples":flight["contact_samples"],
                "elapsed_s":flight["elapsed_s"],
                "min_clearance_m":flight["min_clearance_m"],
                "legacy_rmse_m":flight["navigation_ate_rmse_m"],
                "legacy_precontact_rmse_m":flight["legacy_precontact_rmse_m"],
                "fifteen_rmse_m":flight["eskf15_shadow_rmse_m"],
                "fifteen_precontact_rmse_m":flight["eskf15_shadow_precontact_rmse_m"],
                "fifteen_max_error_m":flight["eskf15_shadow_max_error_m"],
                "fifteen_cov99":flight["eskf15_shadow_cov99"],
                "fifteen_sigma_rms_m":flight["eskf15_shadow_sigma_position_rms_m"],
                "fifteen_lidar3d_accepted":flight["eskf15_lidar3d_accepted"],
                "fifteen_lidar3d_rejected":flight["eskf15_lidar3d_rejected"],
                "fifteen_planar_accepted":flight["eskf15_planar_accepted"],
                "fifteen_planar_rejected":flight["eskf15_planar_rejected"],
                "first_contact":flight["first_contact"]}
            print("PHASE14",json.dumps(record),flush=True)
            rows.append(record)
    original,shadow=rows[:2]
    # Any modification to the legacy physics RNG / control is a failed
    # experimental isolation, even if shadow estimator appears improved.
    same=all(original[k]==shadow[k] for k in (
        "completed","safe","contact_samples","elapsed_s",
        "min_clearance_m","legacy_rmse_m","legacy_precontact_rmse_m"))
    summary={
        "scope":"Phase14 native MuJoCo 15-state body-frame ESKF shadow; synthetic sensors, prior map, no control feedback",
        "same_baseline_closed_loop_flight":same,
        "expected_shadow_flights":2,
        "executed_shadow_flights":2,
        "shadow_flights_safe":sum(x["safe"] for x in rows if x["shadow"]),
        "records":rows}
    (dest/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    if not same:raise AssertionError("Shadow observer changed baseline closed-loop outcomes")
    return summary

if __name__=="__main__":
    evaluate()
