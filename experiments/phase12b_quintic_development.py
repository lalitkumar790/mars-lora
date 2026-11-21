"""Phase 12b paired development: physically track jerk-bounded point-mass reference.

Uses exactly the same development seeds as Phase 12a; this is NOT a new
independent validation. All preflight rejects remain denominator failures.
"""
from __future__ import annotations
import csv,json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

SEEDS=(48221,63047)
METHODS=("shortest","rf_aware")
WIND_N=.35

def evaluate(output="results/phase12b_quintic_development"):
    root=Path(output);root.mkdir(parents=True,exist_ok=True)
    rows=[];failed=[]
    for seed in SEEDS:
        obstacles=INDUSTRIAL+tuple(city(seed))
        for method in METHODS:
            try:
                points=staged_route(method,obstacles,transit_z=10.,
                    localization_sigma_m=.10)
                ref=quintic_reference(points,obstacles,radius_m=.32,
                    sigma_m=.10,clearance_margin_m=.55,
                    max_speed_m_s=1.,max_accel_m_s2=.8,
                    max_jerk_m_s3=1.5,max_duration_s=210.)
            except (ValueError,RuntimeError,AssertionError) as exc:
                failed.append({"seed":seed,"route":method,
                               "stage":"planning","reason":str(exc)})
                print("NO VALID QUINTIC REFERENCE",seed,method,str(exc),flush=True)
                continue
            trace=root/f"flight_{seed}_{method}.csv"
            try:
                flight=run("heavy",duration=240,wind=WIND_N,
                    route=points,trajectory_reference=ref,
                    out=str(trace),wind_phase=(seed%19)*.12,
                    city_seed=seed,controller_mode="legacy",
                    localization_mode="lidar_imu3d_hybrid",
                    dynamic_replanning=False,
                    localization_drift_m_sqrt_s=.03,
                    localization_noise_sigma_m=.02,
                    localization_seed=seed+71,
                    lidar_blackout_window=(45.,75.))
            except (ValueError,RuntimeError,AssertionError) as exc:
                failed.append({"seed":seed,"route":method,
                               "stage":"native_flight","reason":str(exc)})
                print("FLIGHT EXCEPTION",seed,method,str(exc),flush=True)
                continue
            safe=bool(flight["completed"] and
                      flight["contact_samples"]==0 and
                      flight["min_clearance_m"]>=.25)
            flight.update(route=method,safe=safe,
                planned_reference_s=round(ref.duration_s,3),
                ref_speed_m_s=ref.measured_max_speed_m_s,
                ref_accel_m_s2=ref.measured_max_accel_m_s2,
                ref_jerk_m_s3=ref.measured_max_jerk_m_s3)
            rows.append(flight)
            print("QUINTIC",seed,method,"safe",safe,
                  "duration",flight["elapsed_s"],
                  "contacts",flight["contact_samples"],
                  "clearance",flight["min_clearance_m"],flush=True)
    if rows:
        with (root/"trials.csv").open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]))
            w.writeheader();w.writerows(rows)
    summary={"scope":"Development-only 4-scenario MuJoCo reference trial, not heldout, no true SLAM or hardware",
             "expected":len(SEEDS)*len(METHODS),"executed":len(rows),
             "safe":sum(r["safe"] for r in rows),
             "failed":len(SEEDS)*len(METHODS)-sum(r["safe"] for r in rows),
             "contact_failures":sum(r["contact_samples"]>0 for r in rows),
             "clearance_failures":sum(r["min_clearance_m"]<.25 for r in rows),
             "incomplete":sum(not r["completed"] for r in rows),
             "planning_or_flight_exceptions":failed,
             "cases":[{"seed":r["city_seed"],"route":r["route"],"safe":r["safe"],
                       "duration":r["elapsed_s"],"min_clearance":r["min_clearance_m"],
                       "contacts":r["contact_samples"],
                       "navigation_ate_rmse_m":r["navigation_ate_rmse_m"]}
                      for r in rows]}
    (root/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":
    evaluate()
