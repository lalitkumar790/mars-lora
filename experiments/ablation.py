"""Generate a balanced, reproducible 2x2x2 ablation (no fabricated plots)."""
import argparse
import csv
import itertools
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean, stdev
from mars_lora.ablation import Variant,run_ablation

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--seeds",type=int,default=20)
    p.add_argument("--output",default="results/ablation")
    args=p.parse_args()
    if args.seeds < 2:
        p.error("--seeds must be at least two")
    root=Path(args.output)
    root.mkdir(parents=True,exist_ok=True)
    rows=[]
    for speed,shadow in itertools.product((2.,5.,9.),(1.,4.,8.)):
        for seed in range(args.seeds):
            for a,p,s in itertools.product((False,True),repeat=3):
                rows.append(asdict(run_ablation(Variant(a,p,s),seed,speed,shadow)))
    with (root/"trials.csv").open("w",newline="") as file:
        writer=csv.DictWriter(file,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metrics=("pdr","alert_satisfaction","mean_aoi","p95_aoi","outage_fraction","mission_progress_m","tx_airtime_s","tx_energy_j","sf_switches")
    aggregate=[]
    for speed,shadow in itertools.product((2.,5.,9.),(1.,4.,8.)):
        for a,p,s in itertools.product((False,True),repeat=3):
            name=Variant(a,p,s).name
            group=[r for r in rows if r["speed"]==speed and r["shadow"]==shadow and r["policy"]==name]
            record=dict(policy=name,speed=speed,shadow=shadow,n=len(group),adaptive=int(a),priority=int(p),slow=int(s))
            for m in metrics:
                values=[r[m] for r in group]
                record[m+"_mean"]=mean(values)
                record[m+"_sd"]=stdev(values)
            aggregate.append(record)
    with (root/"summary.csv").open("w",newline="") as file:
        writer=csv.DictWriter(file,fieldnames=list(aggregate[0]))
        writer.writeheader()
        writer.writerows(aggregate)
    (root/"manifest.json").write_text(json.dumps({
        "research_design":"full factorial 2x2x2 with shared seeded time-indexed fading",
        "trials":len(rows),"seeds":args.seeds,
        "speed_mps":[2,5,9],"shadow_sigma_db":[1,4,8],
        "policy_factors":["adaptive SF","critical-first queue","20 percent conservative speed reduction"],
        "constant_telemetry_interval_seconds":2,
        "wallclock_duration_seconds":360,
        "constraints":["synthetic raw LoRa","ideal SF receiver synchronization","not hardware-validated","kinematic trajectory","no regulatory duty cycle"]
    },indent=2)+"\n")
    print("Completed",len(rows),"factorial trials; generated",root)

if __name__=="__main__":
    main()
