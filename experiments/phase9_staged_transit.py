"""Phase 9: isolate route-altitude geometry from localization/replanning defects.

Identical physics, vehicle, localization parameters and seeds for original vs
staged transit. This is post-hoc development exploration after failure review.
"""
from __future__ import annotations
import csv,json,argparse
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mujoco_sim.validate_mission import run


def evaluate(out="results/phase9_staged_transit_dev",seeds=(91311,20483)):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    rows=[];plan_errors=[]
    for seed in seeds:
        solids=INDUSTRIAL+tuple(city(seed))
        for method in ("shortest","rf_aware"):
            for stage in (False,True):
                label="staged_10m" if stage else "original_astar"
                try:
                    if stage:
                        points=staged_route(method,solids,transit_z=10.,
                                            localization_sigma_m=.1)
                    else:
                        points=plan(method,solids=solids,
                                    localization_sigma_m=.1).points
                except (ValueError,RuntimeError,AssertionError) as exc:
                    plan_errors.append({"city_seed":seed,"method":method,
                                        "policy":label,"error":str(exc)})
                    continue
                trace=out / f"flight_{seed}_{method}_{label}.csv"
                # Dynamic replanning DISABLED for this diagnostic to isolate
                # original route tracking and altitude staging.
                r=run("heavy",duration=240,wind=.25,route=points,
                      out=str(trace),wind_phase=(seed%19)*.12,
                      city_seed=seed,controller_mode="legacy",
                      localization_mode="lidar_imu3d_hybrid",
                      dynamic_replanning=False,
                      localization_drift_m_sqrt_s=.03,
                      localization_noise_sigma_m=.02,
                      localization_seed=seed+71,
                      lidar_blackout_window=(45.,75.))
                r.update(method=method,policy=label,
                         trace_file=trace.name,
                         safe=bool(r["completed"] and r["contact_samples"]==0
                                   and r["min_clearance_m"]>=.25))
                rows.append(r)
                print(seed,method,label,"safe",r["safe"],
                      "clearance",r["min_clearance_m"],"contacts",r["contact_samples"],
                      "completed",r["completed"],flush=True)
    if rows:
        with (out/"trials.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    summary={"phase":"Post-hoc elevated transit hypothesis; identical physical controller, dynamic replanning disabled",
             "seeds":list(seeds),"expected_missions":len(seeds)*2*2,
             "physically_executed":len(rows),
             "planning_failed":plan_errors,
             "outcomes":[]}
    for label in ("original_astar","staged_10m"):
        group=[r for r in rows if r["policy"]==label]
        summary["outcomes"].append({"policy":label,"expected":len(seeds)*2,
           "executed":len(group),"safe":sum(r["safe"] for r in group),
           "contacts":sum(r["contact_samples"]>0 for r in group),
           "clearance_breaches":sum(r["min_clearance_m"]<.25 for r in group),
           "incomplete":sum(not r["completed"] for r in group),
           "first_events":[{"seed":r["city_seed"],"method":r["method"],
                             "first_clearance":r["first_clearance_violation"],
                             "first_contact":r["first_contact"]}
                            for r in group]})
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/phase9_staged_transit_dev")
    args=parser.parse_args()
    evaluate(args.output)
