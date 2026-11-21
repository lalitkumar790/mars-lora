"""Plot the completed held-out propagation robustness audit without generated values."""
from __future__ import annotations
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

NAMES={"nominal_spatial":"Nominal, spatial fade",
       "mild_urban":"Mild urban",
       "dense_urban":"Dense urban",
       "receiver_mismatch":"Relay mismatch",
       "harsh_interference":"Interference"}
def run(input_file,output_file):
    with open(input_file,newline="") as handle:
        rows=list(csv.DictReader(handle))
    scenarios=list(NAMES)
    fig,axes=plt.subplots(1,2,figsize=(7.0,2.9),sharey=True)
    for ax,(metric,title) in zip(axes,[
        ("pdr","Packet delivery change"),
        ("critical_deadline_ratio","Alert deadline change")]):
        for k,(vehicle,label) in enumerate((("heavy","Skydio model"),
                                           ("crazyflie","Crazyflie model"))):
            subset=[next(r for r in rows if r["scenario"]==s and r["airframe"]==vehicle)
                    for s in scenarios]
            y=np.arange(len(scenarios)) + (k-.5)*.34
            mid=np.array([float(r[f"{metric}_delta"])*100 for r in subset])
            lo=np.array([float(r[f"{metric}_ci95_low"])*100 for r in subset])
            hi=np.array([float(r[f"{metric}_ci95_high"])*100 for r in subset])
            ax.barh(y,mid,height=.32,label=label,alpha=.86)
            ax.errorbar(mid,y,xerr=np.vstack((mid-lo,hi-mid)),fmt="none",
                        ecolor="black",elinewidth=.7,capsize=2)
        ax.axvline(0,color="black",lw=.75)
        ax.set_xlabel("RF-aware minus shortest (pp)",fontsize=8)
        ax.set_title(title,fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(axis="x",alpha=.12)
    axes[0].set_yticks(range(len(scenarios)),[NAMES[s] for s in scenarios],fontsize=8)
    axes[0].invert_yaxis()
    axes[0].legend(fontsize=7,frameon=False,loc="lower right")
    fig.suptitle("Frozen MuJoCo routes, 20 shared spatial-fading seeds per scenario",fontsize=8.5)
    fig.tight_layout()
    target=Path(output_file);target.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(target,bbox_inches="tight")
    fig.savefig(target.with_suffix(".png"),dpi=180,bbox_inches="tight")
    plt.close(fig)
    print("Wrote empirical propagation stress-test figures:",target)
if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--input",default="results/heldout_validation/paired_robustness.csv")
    p.add_argument("--output",default="paper/figures/heldout_robustness.pdf")
    a=p.parse_args();run(a.input,a.output)
