"""Generate paper-ready figures and dashboard JSON from recorded trials.

Never generates fabricated/example data. Requires matplotlib.
"""
import argparse
import csv
import json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",default="results/raw/summary.csv")
    ap.add_argument("--output",default="paper/figures")
    ap.add_argument("--dashboard",default="public")
    args=ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows=list(csv.DictReader(open(args.input,newline="")))
    if not rows:
        raise SystemExit("No experiment results found")
    out=Path(args.output)
    out.mkdir(parents=True,exist_ok=True)
    dashboard=Path(args.dashboard)
    dashboard.mkdir(parents=True,exist_ok=True)
    policies=("fixed","reactive","priority","mars")
    labels={"fixed":"Fixed SF9","reactive":"Reactive SF","priority":"Priority-aware","mars":"MARS-LoRa"}
    specs=[
        ("critical_deadline_ratio_mean","Critical deadline satisfaction","Fraction","deadline_satisfaction"),
        ("p95_aoi_mean","95th percentile AoI","Seconds","aoi_p95"),
        ("outage_fraction_mean","Fraction of time in outage","Fraction","outage"),
        ("tx_energy_j_mean","Estimated transmitter energy","Joules","tx_energy")]
    for metric,title,ylabel,filename in specs:
        fig,ax=plt.subplots(figsize=(6.4,3.4))
        for policy in policies:
            points=sorted((float(r["speed"]),float(r[metric]))
                          for r in rows if r["policy"]==policy and float(r["shadow"])==4)
            ax.plot([p[0] for p in points],[p[1] for p in points],marker="o",label=labels[policy])
        ax.set(xlabel="Nominal UAV speed (m/s)",ylabel=ylabel,title=title)
        ax.grid(alpha=.25)
        ax.legend(fontsize=7,ncol=2)
        fig.tight_layout()
        fig.savefig(out/(filename+".pdf"))
        fig.savefig(out/(filename+".png"),dpi=180)
        fig.savefig(dashboard/(filename+".png"),dpi=180)
        plt.close(fig)
    (dashboard/"results.json").write_text(json.dumps({
        "source":"results/raw/summary.csv",
        "model":"Synthetic event-based LoRa abstraction; kinematic trajectories; NOT a physical flight or field trial",
        "results":[{k:(float(v) if k not in ("policy",) else v) for k,v in row.items()} for row in rows]
    },indent=2)+"\n")
    print("Generated figures from",len(rows),"experiment summary rows")

if __name__=="__main__":
    main()
