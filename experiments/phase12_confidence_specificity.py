"""Specificity check for post-hoc scalar sigma alarm on safe dev flights.

Same 0.50m threshold selected after examining the failed development case;
NOT independent holdout or probabilistic calibration.
"""
from __future__ import annotations
from pathlib import Path
import json
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

CASES=((48221,"shortest"),(63047,"shortest"),(63047,"rf_aware"))

def main():
    output=Path("results/phase12_confidence_specificity_dev")
    output.mkdir(parents=True,exist_ok=True)
    results=[]
    for seed,method in CASES:
        solids=INDUSTRIAL+tuple(city(seed))
        points=staged_route(method,solids,transit_z=10.,localization_sigma_m=.10)
        reference=quintic_reference(points,solids,radius_m=.32,sigma_m=.10,
            clearance_margin_m=.55,max_speed_m_s=1.,
            max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        f=run("heavy",duration=240,wind=.35,route=points,
            city_seed=seed,wind_phase=(seed%19)*.12,
            out=str(output/f"{seed}_{method}.csv"),
            controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
            dynamic_replanning=False,trajectory_reference=reference,
            online_risk_monitor=True,
            localization_drift_m_sqrt_s=.03,
            localization_noise_sigma_m=.02,localization_seed=seed+71,
            lidar_blackout_window=(45.,75.))
        safe=bool(f["completed"] and f["contact_samples"]==0 and
                  f["min_clearance_m"]>=.25)
        item={"seed":seed,"route":method,"safe":safe,"completed":f["completed"],
              "first_confidence_warning":f["loss_of_confidence_first"],
              "confidence_warning_samples":f["loss_of_confidence_samples"],
              "first_geometric_alarm":f["first_risk_alarm"],
              "contact_samples":f["contact_samples"],
              "min_clearance_m":f["min_clearance_m"],
              "elapsed_s":f["elapsed_s"]}
        print("SPECIFICITY",json.dumps(item),flush=True)
        results.append(item)
    summary={"scope":"posthoc known-safe development routes; sigma threshold tuned after failed case, NOT holdout",
             "expected":len(CASES),"safe_flights":sum(x["safe"] for x in results),
             "safe_with_confidence_false_alarm":sum(x["safe"] and x["first_confidence_warning"] is not None for x in results),
             "results":results}
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
if __name__=="__main__":
    main()
