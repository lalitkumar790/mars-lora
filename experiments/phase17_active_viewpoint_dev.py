"""Phase 17: paired native flight development comparison.

The new policy uses known-map body-fixed ray information as a *route score*.
It does not add body-frame LiDAR observations to actual flight estimator.
A negative outcome is scientifically expected if observability changes but
the legacy onboard yaw-only localization has no corresponding measurements.

Planning failures count against expected cases. Never reuse this as holdout.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.active_viewpoint import choose_observable_detour
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

SEEDS=(91311,62731)
ROUTES=("shortest","rf_aware")
POLICIES=("baseline","oracle_observability")
WIND=.25

def evaluate(output="results/phase17_active_viewpoint_dev"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    results=[];errors=[]
    for seed in SEEDS:
        solids=INDUSTRIAL+tuple(city(seed))
        for method in ROUTES:
            try:
                points=staged_route(method,solids,transit_z=10.,
                                    localization_sigma_m=.10)
                proposal=choose_observable_detour(points,solids,
                                      max_reference_s=210.,minimum_gain=.05)
            except (ValueError,RuntimeError,AssertionError) as exc:
                for policy in POLICIES:
                    errors.append({"seed":seed,"route":method,
                         "policy":policy,"stage":"planning",
                         "reason":str(exc)})
                print("NO PLAN",seed,method,str(exc),flush=True)
                continue
            for policy in POLICIES:
                chosen=points if policy=="baseline" else proposal.points
                try:
                    ref=quintic_reference(chosen,solids,radius_m=.32,sigma_m=.10,
                       clearance_margin_m=.55,max_speed_m_s=1.,
                       max_accel_m_s2=.8,max_jerk_m_s3=1.5,
                       max_duration_s=210.)
                    f=run("heavy",duration=240,wind=WIND,city_seed=seed,
                       wind_phase=(seed%19)*.12,route=chosen,
                       trajectory_reference=ref,
                       out=str(out/f"flight_{seed}_{method}_{policy}.csv"),
                       controller_mode="legacy",
                       localization_mode="lidar_imu3d_hybrid",
                       dynamic_replanning=False,
                       localization_drift_m_sqrt_s=.03,
                       localization_noise_sigma_m=.02,
                       localization_seed=seed+71,
                       lidar_blackout_window=(45.,75.))
                except (ValueError,RuntimeError,AssertionError) as exc:
                    errors.append({"seed":seed,"route":method,
                         "policy":policy,"stage":"native_flight",
                         "reason":str(exc)})
                    print("FAILED",seed,method,policy,str(exc),flush=True)
                    continue
                safe=bool(f["completed"] and f["contact_samples"]==0
                          and f["min_clearance_m"]>=.25)
                row={"seed":seed,"route":method,"policy":policy,
                    "safe":safe,"completed":f["completed"],
                    "contact_samples":f["contact_samples"],
                    "min_clearance_m":f["min_clearance_m"],
                    "elapsed_s":f["elapsed_s"],
                    "legacy_ate_rmse_m":f["navigation_ate_rmse_m"],
                    "legacy_precontact_rmse_m":f["legacy_precontact_rmse_m"],
                    "first_contact":f["first_contact"],
                    "route_length_m":proposal.baseline_length_m if policy=="baseline"
                                      else proposal.candidate_length_m,
                    "reference_duration_s":ref.duration_s,
                    "base_observability":proposal.baseline_informative_fraction,
                    "proposal_observability":proposal.candidate_informative_fraction,
                    "inserted_waypoint":proposal.inserted_waypoint,
                    "planner_outcome":proposal.outcome,
                    "candidates_checked":proposal.candidate_count}
                results.append(row)
                print("PHASE17",json.dumps(row),flush=True)
    expected=len(SEEDS)*len(ROUTES)
    summary={"scope":"paired development; perfect prior geometry and oracle nominal body-LiDAR route score; flight uses unchanged yaw-only hybrid",
         "cases_per_policy":expected,"errors":errors,
         "policies":[],"records":results}
    for policy in POLICIES:
        sample=[r for r in results if r["policy"]==policy]
        summary["policies"].append({"policy":policy,"expected":expected,
             "executed":len(sample),"safe":sum(r["safe"] for r in sample),
             "failed":expected-sum(r["safe"] for r in sample),
             "incomplete":sum(not r["completed"] for r in sample),
             "contact_failures":sum(r["contact_samples"]>0 for r in sample)})
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":evaluate()
