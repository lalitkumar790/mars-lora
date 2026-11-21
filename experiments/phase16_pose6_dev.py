"""Development-only 6-DoF local range information audit.

Compare separate body-fixed synthetic scan geometry on the two characterized
MuJoCo missions. Both are pre-selected development cases, not holdout.
No full-pose LiDAR update or any new control action is permitted.
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


def evaluate(output="results/phase16_pose6_dev"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for seed,method,wind in CASES:
        solids=INDUSTRIAL+tuple(city(seed))
        points=staged_route(method,solids,transit_z=10.,localization_sigma_m=.1)
        reference=quintic_reference(points,solids,radius_m=.32,
            sigma_m=.1,clearance_margin_m=.55,max_speed_m_s=1.,
            max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        for enabled in ((False,True) if seed==91311 else (True,)):
            flight=run("heavy",duration=240,wind=wind,city_seed=seed,
                wind_phase=(seed%19)*.12,route=points,
                trajectory_reference=reference,pose6_shadow=enabled,
                eskf15_shadow=True,controller_mode="legacy",
                localization_mode="lidar_imu3d_hybrid",dynamic_replanning=False,
                out=str(out/f"flight_{seed}_{method}_{'six_dof' if enabled else 'baseline'}.csv"),
                localization_drift_m_sqrt_s=.03,localization_noise_sigma_m=.02,
                localization_seed=seed+71,lidar_blackout_window=(45.,75.))
            first=flight["first_contact"]
            cutoff=first["t_s"] if first else float("inf")
            scans=flight["pose6_observability_timeline"]
            before=[item for item in scans if item["t_s"]<cutoff]
            item={"seed":seed,"route":method,"pose6_shadow":enabled,
                "completed":flight["completed"],
                "contacts":flight["contact_samples"],
                "min_clearance_m":flight["min_clearance_m"],
                "elapsed_s":flight["elapsed_s"],
                "legacy_rmse_m":flight["navigation_ate_rmse_m"],
                "eskf15_precontact_rmse_m":flight["eskf15_shadow_precontact_rmse_m"],
                "first_contact_t_s":first["t_s"] if first else None,
                "precontact_scans":len(before),
                "precontact_full_rank6":sum(x["full_rank"] for x in before),
                "precontact_rank6_histogram":{
                    str(k):sum(x["rank6"]==k for x in before) for k in range(7)},
                "precontact_translation_rank3":sum(x["rank_xyz"]==3 for x in before),
                "precontact_rotation_rank3":sum(x["rank_rotation"]==3 for x in before)}
            (out/f"pose6_timeline_{seed}_{method}.json").write_text(
                json.dumps(scans,indent=2)+"\n")
            rows.append(item)
            print("POSE6",json.dumps(item),flush=True)
    a,b=rows[:2]
    fields=("completed","contacts","min_clearance_m","elapsed_s",
            "legacy_rmse_m","eskf15_precontact_rmse_m")
    baseline_preserved=all(a[k]==b[k] for k in fields)
    summary={"scope":"Passive separate synthetic body-fixed LiDAR; 2 known development scenarios; no estimator correction",
             "baseline_preserved":baseline_preserved,"records":rows}
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    if not baseline_preserved:
        raise AssertionError("Additional sensor changed original flight/estimator")
    return summary

if __name__=="__main__":evaluate()
