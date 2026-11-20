"""Evaluate browser GPS-denied SLAM using independently exported MuJoCo truth.

Inputs:
- mars-lora-lidar-odom.jsonl (LiDAR and drift-only odom + map-corrected estimate).
- mars-lora-mujoco-flight.csv (full physics truth, same browser session).
The estimator NEVER reads this truth; use only for offline evaluation.
Output: JSON descriptive errors, no claims without actual paired logs.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from statistics import mean, median

def percentile(arr,p):
    s=sorted(arr)
    if not s:return float("nan")
    idx=(len(s)-1)*p
    k=int(idx)
    return s[k]*(1-(idx-k))+s[min(len(s)-1,k+1)]*(idx-k)

def metric(records,gt,tolerance=.3,window=10):
    times=[r[0] for r in gt]
    aligned=[]
    for item in records:
        t=float(item["t"])
        j=bisect.bisect_left(times,t)
        candidates=[i for i in (j-1,j) if 0<=i<len(gt)]
        if not candidates:continue
        k=min(candidates,key=lambda i:abs(times[i]-t))
        if abs(times[k]-t)>tolerance:continue
        truth=gt[k]
        est=item["estimated"]
        odom=item["odom"]
        aligned.append(dict(t=t,truth=(truth[1],truth[2]),
                            est=(float(est["x"]),float(est["y"])),
                            odom=(float(odom["x"]),float(odom["y"])),
                            beams=len(item["ranges"]),
                            hits=sum(float(d)<float(item["range_max"])-.05 for d in item["ranges"])))
    if len(aligned)<3:
        raise ValueError("No synchronized LiDAR/ground-truth measurements. Export both logs from the same browser flight.")
    output={"aligned_samples":len(aligned),"duration_s":aligned[-1]["t"]-aligned[0]["t"]}
    aligned_times=[r['t'] for r in aligned]
    for method in ("odom","est"):
        errors=[math.dist(r[method],r["truth"]) for r in aligned]
        output[method+"_ate_rmse_m"]=math.sqrt(mean([e*e for e in errors]))
        output[method+"_ate_p95_m"]=percentile(errors,.95)
        output[method+"_ate_median_m"]=median(errors)
        relative=[]
        for i,a in enumerate(aligned):
            t=a["t"]
            j=bisect.bisect_left(aligned_times,t+window)
            if j>=len(aligned):continue
            b=aligned[j]
            dt=(b["truth"][0]-a["truth"][0],b["truth"][1]-a["truth"][1])
            de=(b[method][0]-a[method][0],b[method][1]-a[method][1])
            relative.append(math.dist(de,dt))
        output[method+f"_rpe_{window}s_rmse_m"]=(math.sqrt(mean([x*x for x in relative])) if relative else None)
    output["mean_valid_range_beams"]=mean([r["hits"] for r in aligned])
    output["mean_observable_fraction"]=mean([r["hits"]/r["beams"] for r in aligned])
    output["evaluation_scope"]="Planar XY, same-session simulated MuJoCo truth; no field measurements"
    return output

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--slam",type=Path,required=True)
    ap.add_argument("--flight",type=Path,required=True)
    ap.add_argument("--output",type=Path,default=Path("results/slam-eval/metrics.json"))
    args=ap.parse_args()
    records=[json.loads(line) for line in args.slam.read_text().splitlines() if line.strip()]
    with args.flight.open(newline="") as f:
        table=csv.DictReader(f)
        truth=[]
        for row in table:
            t=row.get("time_s") or row.get("t_s")
            if t is not None:
                truth.append((float(t),float(row["x_m"]),float(row["y_m"])))
    truth.sort(key=lambda x:x[0])
    metrics=metric(records,truth)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(metrics,indent=2)+"\n")
    print(json.dumps(metrics,indent=2))
if __name__=="__main__":
    main()
