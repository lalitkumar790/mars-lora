"""Exploratory preflight route gate using previously completed MuJoCo+LoRa trials.

This analysis does NOT claim independent confirmatory testing. The heuristic
was designed after inspecting the five scenario results, so generalization
must be checked on genuinely new scenes and channel measurements.
"""
from __future__ import annotations
import argparse,csv,json,math
from collections import Counter
from pathlib import Path
from statistics import mean,stdev
from experiments.heldout_validation import PROFILES,STATIONS
from mars_lora.route_gate import choose_route,simulated_pilot

METRICS=("pdr","critical_deadline_ratio","aoi_p95_s",
         "outage_fraction","tx_energy_j","mission_duration_s")
def run(input_file,output_dir):
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    with open(input_file,newline="") as f:rows=list(csv.DictReader(f))
    data={}
    for r in rows:
        key=(r["scenario"],r["airframe"],int(r["seed"]))
        data.setdefault(key,{})[r["route"]]=r
    trials=[]
    for (scenario,airframe,seed),pair in sorted(data.items()):
        if set(pair)!={"shortest","rf_aware"}:
            raise ValueError("Incomplete cross-route matched pair")
        pilot=simulated_pilot(PROFILES[scenario],STATIONS[scenario],seed)
        choice=choose_route(pilot["pilot_rssi_dbm"],pilot["observed_channel_busy"])
        result=dict(scenario=scenario,airframe=airframe,seed=seed,
                    selected_route=choice,**pilot)
        for metric in METRICS:
            result["gate_"+metric]=pair[choice][metric]
            result["shortest_"+metric]=pair["shortest"][metric]
            result["rf_aware_"+metric]=pair["rf_aware"][metric]
        trials.append(result)
    if not trials:raise ValueError("No pilot choices")
    with (out/"gate_trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(trials[0]));w.writeheader();w.writerows(trials)
    summary=[]
    for scenario in PROFILES:
        for airframe in ("heavy","crazyflie"):
            xs=[r for r in trials if r["scenario"]==scenario and r["airframe"]==airframe]
            if not xs:continue
            record=dict(scenario=scenario,airframe=airframe,n=len(xs),
                        selected_shortest=sum(r["selected_route"]=="shortest" for r in xs),
                        selected_rf_aware=sum(r["selected_route"]=="rf_aware" for r in xs))
            for m in METRICS:
                gate=[float(r["gate_"+m]) for r in xs]
                for policy in ("shortest","rf_aware"):
                    baseline=[float(r[policy+"_"+m]) for r in xs]
                    diffs=[a-b for a,b in zip(gate,baseline)]
                    record[m+"_gate_mean"]=mean(gate)
                    record[m+"_"+policy+"_mean"]=mean(baseline)
                    record[m+"_gate_minus_"+policy]=mean(diffs)
            summary.append(record)
    with (out/"gate_summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    (out/"manifest.json").write_text(json.dumps({
      "status":"Exploratory post-hoc heuristic; not confirmatory research",
      "policy":"Observable RSSI and measured channel busy slots only",
      "thresholds":{"sf9_rssi_margin_db":10,"channel_busy_threshold":.25,
                    "cca_pilot_samples":32},
      "physics":"Routes from prior verified native MuJoCo runs, not newly flown",
      "caveats":[
          "Thresholds chosen after seeing held-out scenario results; selection bias risk",
          "Synthetic pilot RSSI and channel-busy sampling, not radio hardware",
          "Decision evaluated on the same scenarios that motivated the heuristic",
          "Same 3D obstacle/radio loss abstraction still underlies evaluation",
          "Needs blind city/propagation trials before any paper superiority claim"
      ]},indent=2)+"\n")
    for row in summary:
        print(row["scenario"],row["airframe"],"short",row["selected_shortest"],
              "RF",row["selected_rf_aware"],"gate PDR",round(row["pdr_gate_mean"],3),
              "alert",round(row["critical_deadline_ratio_gate_mean"],3))
    return summary

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--input",default="results/heldout_validation/trials.csv")
    p.add_argument("--output",default="results/preflight_gate")
    args=p.parse_args()
    run(args.input,args.output)
