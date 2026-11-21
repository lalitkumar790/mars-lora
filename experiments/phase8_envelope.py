"""Exploratory development-city ablation: V5 vs conservative stopping envelope.

Not confirmatory; geometry and envelope parameters chosen after V5 failures.
Do not include these exploratory results in held-out scientific claims.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run


def evaluate(output,city_seeds=(91311,20483)):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    rows=[]
    errors=[]
    for seed in city_seeds:
        solids=INDUSTRIAL+tuple(city(seed))
        try:
            paths={method:plan(method,solids=solids,localization_sigma_m=.10).points
                   for method in ("shortest","rf_aware")}
        except (RuntimeError,ValueError,AssertionError) as exc:
            errors.append({"seed":seed,"error":str(exc)})
            continue
        for method,waypoints in paths.items():
            for envelope in (False,True):
                x=run("heavy",duration=240,wind=.25,
                      wind_phase=(seed%19)*.12,city_seed=seed,
                      controller_mode="legacy",route=waypoints,
                      localization_mode="lidar_imu3d_hybrid",
                      dynamic_replanning=True,replan_stopping_envelope=envelope,
                      localization_drift_m_sqrt_s=.03,
                      localization_noise_sigma_m=.02,
                      localization_seed=seed+71,
                      lidar_blackout_window=(45.,75.))
                x.update(route=method,envelope=envelope,
                         safe=bool(x["completed"] and x["contact_samples"]==0
                                   and x["min_clearance_m"]>=.25))
                rows.append(x)
                print(seed,method,"envelope",envelope,"safe",x["safe"],
                      "replans",x["map_replans"],
                      "rejected",x["stopping_envelope_rejections"],flush=True)
    with (output/"trials.csv").open("w",newline="") as f:
        if rows:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    expected=len(city_seeds)*2*2
    summary={"expected_missions":expected,"executed":len(rows),
             "planning_failures":expected-len(rows),"planner_errors":errors,
             "matched_comparisons":[],
             "scope":"Post-hoc development-city MuJoCo ablation, not independent validation."}
    for flag in (False,True):
        r=[x for x in rows if x["envelope"]==flag]
        summary["matched_comparisons"].append({
            "stopping_envelope":flag,"safe":sum(x["safe"] for x in r),
            "expected":len(city_seeds)*2,"executed":len(r),
            "contact_failures":sum(x["contact_samples"]>0 for x in r),
            "clearance_failures":sum(x["min_clearance_m"]<.25 for x in r),
            "incomplete":sum(not x["completed"] for x in r),
            "replans":sum(x["map_replans"] for x in r),
            "replan_failures":sum(x["map_replan_failures"] for x in r),
            "envelope_rejections":sum(x["stopping_envelope_rejections"] for x in r)})
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    return summary


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/phase8_envelope_dev")
    parser.add_argument("--city-seeds",type=int,nargs="+",default=[91311,20483])
    args=parser.parse_args()
    print(json.dumps(evaluate(args.output,tuple(args.city_seeds)),indent=2))
