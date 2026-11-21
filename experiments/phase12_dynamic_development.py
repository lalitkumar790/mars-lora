"""Phase 12 paired development experiment (not a new holdout).

Predeclared development seeds exclude Phases 8–11 development and all
previous holdouts. All rejected trajectories count in mission denominators.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.transit_route import staged_route
from mars_lora.trajectory_dynamics import build_trajectory
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

DEVELOPMENT_SEEDS=(48221,63047)
METHODS=("shortest","rf_aware")
WIND=.35
LIMIT=.65

def evaluate(output="results/phase12_dynamic_development"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[];errors=[]
    for seed in DEVELOPMENT_SEEDS:
        solids=INDUSTRIAL+tuple(city(seed))
        for method in METHODS:
            for policy in ("staged_10m","dynamic_timed"):
                try:
                    if policy=="staged_10m":
                        points=staged_route(method,solids,transit_z=10.,
                                             localization_sigma_m=.10)
                        limits=None
                    else:
                        timed=build_trajectory(method,solids,sigma_m=.12,
                            geometric_margin_m=.55,max_speed_m_s=1.0,
                            braking_m_s2=.8,motor_latency_s=.2,
                            wind_accel_bound_m_s2=.15)
                        points=timed.points
                        limits=timed.waypoint_speed_limits_m_s
                except (ValueError,RuntimeError,AssertionError) as exc:
                    errors.append({"seed":seed,"route":method,"policy":policy,
                                   "stage":"planning","error":str(exc)})
                    print("NO PLAN",seed,method,policy,str(exc),flush=True)
                    continue
                trace=out/f"{seed}_{method}_{policy}.csv"
                try:
                    r=run("heavy",duration=240,wind=WIND,route=points,
                          out=str(trace),wind_phase=(seed%19)*.12,city_seed=seed,
                          controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
                          dynamic_replanning=False,altitude_slew_m_s=LIMIT,
                          trajectory_speed_limits=limits,
                          localization_drift_m_sqrt_s=.03,
                          localization_noise_sigma_m=.02,
                          localization_seed=seed+71,lidar_blackout_window=(45.,75.))
                except (ValueError,RuntimeError,AssertionError) as exc:
                    errors.append({"seed":seed,"route":method,"policy":policy,
                                   "stage":"flight","error":str(exc)})
                    print("FAILED RUN",seed,method,policy,str(exc),flush=True)
                    continue
                r.update(policy=policy,route=method,wind_n=WIND,
                         safe=bool(r["completed"] and r["contact_samples"]==0
                                   and r["min_clearance_m"]>=.25))
                rows.append(r)
                print("DEV",seed,method,policy,"safe",r["safe"],
                      "contacts",r["contact_samples"],
                      "clearance",r["min_clearance_m"],
                      "done",r["completed"],"duration",r["elapsed_s"],flush=True)
    if rows:
        with (out/"trials.csv").open("w",newline="") as fp:
            w=csv.DictWriter(fp,fieldnames=list(rows[0]))
            w.writeheader();w.writerows(rows)
    expected=len(DEVELOPMENT_SEEDS)*len(METHODS)
    result={"type":"phase12 posthoc development (not holdout)",
        "development_city_seeds":list(DEVELOPMENT_SEEDS),
        "wind_force_n":WIND,"failures":errors,"policies":[]}
    for policy in ("staged_10m","dynamic_timed"):
        group=[x for x in rows if x["policy"]==policy]
        result["policies"].append({"name":policy,"expected":expected,
            "executed":len(group),"safe":sum(x["safe"] for x in group),
            "contact_failures":sum(x["contact_samples"]>0 for x in group),
            "clearance_failures":sum(x["min_clearance_m"]<.25 for x in group),
            "incomplete":sum(not x["completed"] for x in group),
            "events":[{"city_seed":x["city_seed"],"method":x["route"],
                       "safe":x["safe"],"first_contact":x["first_contact"],
                       "first_clearance":x["first_clearance_violation"]}
                      for x in group]})
    (out/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2),flush=True)
    return result

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/phase12_dynamic_development")
    args=parser.parse_args()
    evaluate(args.output)
