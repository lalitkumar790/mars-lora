"""Held-out, map-known UAV LoRa route robustness audit.

The A* planner remains frozen; all evaluations use EXISTING MuJoCo flight
CSVs. Held-out propagation settings are deliberately not provided to A*.
Shadowing is one common spatial field per random seed, so path A and path B
experience the same environment at each shared 3-D location.

This is a *model-mismatch stress test*, not independent field validation:
ray/AABB obstructions and sigmoid reception still come from our simulator.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
from statistics import mean,stdev
from experiments.replay_mujoco_radio import load_trajectory,trial
from mars_lora.rf_astar import GCS

PROFILES={
    "nominal_spatial":dict(name="nominal_spatial",path_loss_exponent=4.,
                           obstruction_db=17.,rx_bias_db=0.,interference_fraction=.1),
    "mild_urban":dict(name="mild_urban",path_loss_exponent=3.2,
                      obstruction_db=9.,rx_bias_db=0.,interference_fraction=.1),
    "dense_urban":dict(name="dense_urban",path_loss_exponent=4.4,
                       obstruction_db=26.,rx_bias_db=0.,interference_fraction=.1),
    "receiver_mismatch":dict(name="receiver_mismatch",path_loss_exponent=4.,
                             obstruction_db=17.,rx_bias_db=-7.,interference_fraction=.16),
    "harsh_interference":dict(name="harsh_interference",path_loss_exponent=3.7,
                              obstruction_db=19.,rx_bias_db=0.,interference_fraction=.35)
}
STATIONS={
    "nominal_spatial":GCS,
    "mild_urban":GCS,
    "dense_urban":GCS,
    "receiver_mismatch":(185.,155.,75.),
    "harsh_interference":GCS,
}
METRICS=("pdr","critical_deadline_ratio","aoi_p95_s","outage_fraction",
         "nlos_fraction","tx_energy_j","mission_duration_s")
AIRFRAMES=("heavy","crazyflie")
ROUTES=("shortest","rf_aware")

def evaluate(input_dir,output_dir,seeds=20,profiles=PROFILES):
    if seeds<2:raise ValueError("Minimum two seeds")
    directory=Path(input_dir)
    out=Path(output_dir)
    out.mkdir(parents=True,exist_ok=True)
    traces={(airframe,route):load_trajectory(directory/f"{airframe}_{route}_flight.csv")
            for airframe in AIRFRAMES for route in ROUTES}
    trials=[]
    for profile_name,spec in profiles.items():
        station=STATIONS[profile_name]
        for airframe in AIRFRAMES:
            for seed in range(seeds):
                for route in ROUTES:
                    # identical packet policy; only the executed route differs.
                    result=trial(traces[(airframe,route)],"priority-adaptive",seed,
                                 station,shadow_sigma=5.,step=.25,
                                 profile=spec,spatial_fading=True)
                    result.update(airframe=airframe,route=route,
                                  scenario=profile_name)
                    trials.append(result)
    with (out/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(trials[0]))
        w.writeheader();w.writerows(trials)
    pairs={}
    for row in trials:
        key=(row["scenario"],row["airframe"],int(row["seed"]))
        pairs.setdefault(key,{})[row["route"]]=row
    summary=[]
    for name in profiles:
        for airframe in AIRFRAMES:
            subset=[pair for (scenario,vehicle,_),pair in sorted(pairs.items())
                    if scenario==name and vehicle==airframe]
            if not subset or any(set(pair)!=set(ROUTES) for pair in subset):
                raise RuntimeError("Non-paired scenario detected")
            record={"scenario":name,"airframe":airframe,"paired_n":len(subset)}
            for metric in METRICS:
                deltas=[float(pair["rf_aware"][metric])-float(pair["shortest"][metric])
                        for pair in subset]
                v0=[float(pair["shortest"][metric]) for pair in subset]
                v1=[float(pair["rf_aware"][metric]) for pair in subset]
                change=mean(deltas)
                uncertainty=1.96*stdev(deltas)/math.sqrt(len(deltas))
                record[f"{metric}_shortest_mean"]=mean(v0)
                record[f"{metric}_rf_mean"]=mean(v1)
                record[f"{metric}_delta"]=change
                record[f"{metric}_ci95_low"]=change-uncertainty
                record[f"{metric}_ci95_high"]=change+uncertainty
            summary.append(record)
    with (out/"paired_robustness.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]))
        w.writeheader();w.writerows(summary)
    (out/"manifest.json").write_text(json.dumps({
        "experiment":"Frozen RF-aware 3D routes evaluated under unseen propagation parameters",
        "evidence":"native MuJoCo output replay; no new flights; assumed synthetic radio physics",
        "seeds":seeds,
        "scenarios":PROFILES,
        "ground_stations":{k:list(v) for k,v in STATIONS.items()},
        "paired_comparison":"RF-aware route vs shortest route, identical adaptive priority policy",
        "fading":"shared seeded 3D Gaussian interpolated spatial shadow field",
        "limitations":[
            "not a calibrated propagation model or independent physics engine",
            "same deterministic vehicle trajectories in all scenarios",
            "receiver change does not trigger online trajectory re-planning",
            "packet-generation horizon differs across route duration",
            "95% intervals are conditional on synthetic fading draws and one fixed city",
            "spatial Gaussian field is smooth with location-dependent SD"
        ]
    },indent=2)+"\n")
    print(json.dumps([{k:r[k] for k in ("scenario","airframe","paired_n",
                                       "pdr_shortest_mean","pdr_rf_mean","pdr_delta",
                                       "pdr_ci95_low","pdr_ci95_high",
                                       "critical_deadline_ratio_delta")}
                      for r in summary],indent=2))
    return summary

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--input",default="results/rf_navigation")
    parser.add_argument("--output",default="results/heldout_validation")
    parser.add_argument("--seeds",type=int,default=20)
    args=parser.parse_args()
    evaluate(args.input,args.output,args.seeds)
