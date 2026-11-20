"""Reproducible paired comparisons for fully executed 3D UAV RF-route experiments.

The comparisons are paired by airframe, shadowing level, and radio seed.
95% intervals use a simple normal approximation around matched differences;
they are *not* multiple-testing corrected or evidence of field performance.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
from statistics import mean,stdev

METRICS=("pdr","critical_deadline_ratio","aoi_p95_s","tx_energy_j",
         "mission_duration_s","nlos_fraction")
COMPARISONS=(("R2_RF_route_adaptive","S1_shortest_adaptive"),
             ("S1_shortest_adaptive","S0_shortest_fixed"))

def analyze(path):
    with open(path,newline="") as f:
        rows=list(csv.DictReader(f))
    grouped={}
    for r in rows:
        key=(r["airframe"],r["shadow_sigma_db"],int(r["seed"]))
        methods=grouped.setdefault(key,{})
        if r["condition"] in methods:
            raise ValueError("Duplicate condition for paired seed: "+str(key))
        methods[r["condition"]]=r
    results=[]
    for airframe in sorted({k[0] for k in grouped}):
        for sigma in sorted({k[1] for k in grouped},key=float):
            for newer,older in COMPARISONS:
                pairs=[(methods[newer],methods[older])
                       for (a,s,_),methods in sorted(grouped.items())
                       if a==airframe and s==sigma and newer in methods and older in methods]
                if len(pairs)<2:continue
                record=dict(airframe=airframe,shadow_sigma_db=float(sigma),treatment=newer,
                            control=older,paired_n=len(pairs))
                for metric in METRICS:
                    d=[float(a[metric])-float(b[metric]) for a,b in pairs]
                    mu=mean(d)
                    se=stdev(d)/math.sqrt(len(d))
                    record[f"{metric}_delta"]=mu
                    record[f"{metric}_ci95_low"]=mu-1.96*se
                    record[f"{metric}_ci95_high"]=mu+1.96*se
                results.append(record)
    return results

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--trials",default="results/rf_navigation/trials.csv")
    parser.add_argument("--output",default="results/rf_navigation/paired_effects.csv")
    args=parser.parse_args()
    result=analyze(args.trials)
    if not result:raise SystemExit("No complete paired comparisons")
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    with open(args.output,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(result[0]));w.writeheader();w.writerows(result)
    print(json.dumps(result,indent=2))
if __name__=="__main__":main()
