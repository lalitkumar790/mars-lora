"""Draw evidence-only cross-layer figures from recorded MuJoCo->LoRa CSV."""
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--summary",default="results/crosslayer/summary.csv")
    p.add_argument("--output",default="paper/figures/crosslayer_policies.pdf")
    args=p.parse_args()
    with open(args.summary,newline="") as file:rows=list(csv.DictReader(file))
    rows=[r for r in rows if r["airframe"]=="heavy" and float(r["shadow_sigma_db"])==4]
    policy=[("fixed","Fixed SF9"),("adaptive","Adaptive SF"),("priority-adaptive","Adaptive + priority")]
    scenarios=[("5.0","Ground GCS"),("120.0","Elevated relay")]
    fig,axs=plt.subplots(2,2,figsize=(6.8,4.6))
    metrics=[("pdr_mean","Delivery probability"),("critical_deadline_ratio_mean","Critical timely ratio"),
             ("aoi_p95_s_mean","95th percentile AoI (s)"),("tx_energy_j_mean","Transmitter energy proxy (J)")]
    for ax,(key,label) in zip(axs.flat,metrics):
        for j,(p,label_p) in enumerate(policy):
            y=[float(next(row for row in rows if row["policy"]==p and row["gcs_z"]==z)[key])
               for z,_ in scenarios]
            x=[k+(j-1)*.23 for k in range(len(scenarios))]
            ax.bar(x,y,width=.22,label=label_p)
        ax.set_xticks(range(len(scenarios)),[name for _,name in scenarios],fontsize=8)
        ax.set_ylabel(label,fontsize=8)
        ax.tick_params(axis="y",labelsize=8)
        ax.grid(axis="y",alpha=.2)
    axs[0,0].legend(loc="upper left",fontsize=6,frameon=False)
    fig.suptitle("3D MuJoCo flight replay / synthetic LoRa, 20 paired seeds, shadow = 4 dB",fontsize=9)
    fig.tight_layout()
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,bbox_inches="tight")
    fig.savefig(str(Path(args.output).with_suffix(".png")),dpi=170,bbox_inches="tight")
    plt.close(fig)
    print("Saved verified cross-layer figure",args.output)
if __name__=="__main__":main()
