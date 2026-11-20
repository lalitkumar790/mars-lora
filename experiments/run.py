"""Execute paired-seed experiment matrix. All outputs are empirical simulation outputs."""
import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
from statistics import mean, stdev
from mars_lora.channel import run_trial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default="results/raw")
    ap.add_argument("--seeds", type=int, default=20)
    args = ap.parse_args()
    if args.seeds < 2:
        ap.error("At least two seeds are required for variance estimates.")
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    rows = []
    for speed in (2.0, 5.0, 9.0):
        for shadow in (1.0, 4.0, 8.0):
            for seed in range(args.seeds):
                for policy in ("fixed", "reactive", "priority", "mars"):
                    outcome = run_trial(policy, speed, shadow, seed)
                    record = asdict(outcome)
                    record["pdr"] = outcome.received/outcome.generated
                    record["critical_deadline_ratio"] = (outcome.critical_on_time/outcome.critical_generated
                                                         if outcome.critical_generated else float("nan"))
                    rows.append(record)
    fieldnames = list(rows[0])
    with (root/"trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames)
        w.writeheader()
        w.writerows(rows)
    metrics = ("pdr","critical_deadline_ratio","mean_aoi","p95_aoi",
               "outage_fraction","tx_energy_j","mission_duration_s","total_airtime_s")
    summary = []
    for speed in (2.0,5.0,9.0):
        for shadow in (1.0,4.0,8.0):
            for policy in ("fixed","reactive","priority","mars"):
                group = [r for r in rows if (r["speed"],r["shadow"],r["policy"])==(speed,shadow,policy)]
                d = {"speed":speed,"shadow":shadow,"policy":policy,"n":len(group)}
                for metric in metrics:
                    values = [r[metric] for r in group if math.isfinite(r[metric])]
                    d[metric+"_mean"] = mean(values)
                    d[metric+"_sd"] = stdev(values) if len(values)>1 else 0.0
                summary.append(d)
    with (root/"summary.csv").open("w",newline="") as f:
        w = csv.DictWriter(f, list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    manifest = {
        "model":"synthetic raw LoRa link abstraction; not validated RF/LoRaWAN",
        "seeds":args.seeds, "trial_count":len(rows),
        "policies":["fixed","reactive","priority","mars"],
        "speeds_mps":[2,5,9], "shadow_sigma_db":[1,4,8],
        "duration_s":360, "dt_s":0.5,
        "caveats":["ideal SF synchronization","synthetic contention","kinematic trajectory",
                   "TX-only radio-energy proxy","no mission safety proof"]
    }
    (root/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("Wrote",len(rows),"trials to",root)

if __name__ == "__main__":
    main()
