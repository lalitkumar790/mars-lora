"""Figures from the executed MuJoCo RF-aware routing experiment only.

No synthetic placeholder data are used. The route overlay reads the planned
A* JSON and real MuJoCo flight CSV; error bars are across seeded radio trials.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mars_lora.rf_astar import SOLIDS,GCS

def rows(path):
    with open(path,newline="") as f:return list(csv.DictReader(f))

def save(fig,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,bbox_inches="tight")
    fig.savefig(path.with_suffix(".png"),dpi=180,bbox_inches="tight")
    plt.close(fig)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--results",default="results/rf_navigation")
    p.add_argument("--figures",default="paper/figures")
    args=p.parse_args()
    src=Path(args.results);out=Path(args.figures)
    routes=json.loads((src/"routes.json").read_text())
    fig,ax=plt.subplots(figsize=(4.3,3.6))
    for b in SOLIDS:
        x,y,z,hx,hy,hz=b
        if x+hx<-36 or x-hx>8 or y+hy<-8 or y-hy>32:continue
        ax.add_patch(Rectangle((x-hx,y-hy),2*hx,2*hy,color="#a8acb2",alpha=.45,lw=0))
    for name,label,color in (("shortest","Geometric A*","#bf6249"),
                              ("rf_aware","RF-aware A*","#276eae")):
        points=routes[name]["points"]
        ax.plot([p[0] for p in points],[p[1] for p in points],
                lw=1.9,color=color,alpha=.87,label=label)
        trace=rows(src/f"heavy_{name}_flight.csv")
        # Thin, actual executed MuJoCo curve: *not* an ideal planned line.
        ax.plot([float(v["x_m"]) for v in trace],[float(v["y_m"]) for v in trace],
                color=color,alpha=.45,lw=.8,linestyle=":")
    ax.scatter([0,-30],[0,24],s=25,color="black",zorder=6)
    ax.annotate("Start",(0,0),xytext=(3,-1.5),fontsize=7)
    ax.annotate("Goal",(-30,24),xytext=(-29,27),fontsize=7)
    ax.set(xlabel="X (m)",ylabel="Y (m)",xlim=(-36,8),ylim=(-8,32))
    ax.set_aspect("equal",adjustable="box")
    ax.grid(alpha=.18)
    ax.legend(loc="lower left",fontsize=7,frameon=False)
    fig.tight_layout()
    save(fig,out/"rf_aware_routes.pdf")
    summary=rows(src/"summary.csv")
    conds=("S0_shortest_fixed","S1_shortest_adaptive","R2_RF_route_adaptive")
    labels=("Short/fixed","Short/adaptive","RF path/adaptive")
    colors=("#a89c8c","#bf6249","#276eae")
    fig,axs=plt.subplots(1,2,figsize=(6.7,2.55))
    for airframe,ax in zip(("heavy","crazyflie"),axs):
        ys=[];errs=[]
        for cond in conds:
            r=next(x for x in summary if x["airframe"]==airframe and
                   float(x["shadow_sigma_db"])==4. and x["condition"]==cond)
            ys.append(100*float(r["pdr_mean"]))
            errs.append(100*float(r["pdr_sd"]))
        ax.bar(range(3),ys,yerr=errs,capsize=3,color=colors,alpha=.85,width=.66)
        ax.set_xticks(range(3),labels,rotation=18,ha="right",fontsize=7)
        ax.set_ylim(0,105)
        ax.set_ylabel("Delivered packets (%)",fontsize=8)
        ax.set_title("Skydio-based" if airframe=="heavy" else "Crazyflie",fontsize=9)
        ax.grid(axis="y",alpha=.2)
        ax.tick_params(axis="y",labelsize=8)
    fig.suptitle("Native MuJoCo route execution + raw-LoRa replay (20 seeds, 4 dB)",fontsize=9)
    fig.tight_layout()
    save(fig,out/"rf_aware_tradeoff.pdf")
    print("Saved paper figures based only on verified executed flight and radio CSV")

if __name__=="__main__":main()
