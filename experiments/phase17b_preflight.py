"""Phase17b preflight audit: count why 3D viewpoint detours cannot be used.

No new flight claims. Reuse *development only* seeds, including Phase 17
failed preflight seed, and report every rejected city/route combination.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.active_viewpoint import choose_observable_detour

SEEDS=(91311,48221,63047,62731)
METHODS=("shortest","rf_aware")

def main():
    results=[]
    for seed in SEEDS:
        obstacles=INDUSTRIAL+tuple(city(seed))
        for method in METHODS:
            try:
                route=staged_route(method,obstacles,transit_z=10.,
                                   localization_sigma_m=.10)
                chosen=choose_observable_detour(route,obstacles)
                record={"seed":seed,"route":method,"status":chosen.outcome,
                        "tested":chosen.candidate_count,
                        "rejected":chosen.rejected_count,
                        "reasons":chosen.rejection_reasons,
                        "baseline_information_fraction":chosen.baseline_informative_fraction,
                        "best_rejected_information_fraction":chosen.best_rejected_score,
                        "chosen_information_fraction":chosen.candidate_informative_fraction,
                        "extra_length_m":chosen.candidate_length_m-chosen.baseline_length_m,
                        "extra_duration_s":chosen.candidate_duration_s-chosen.baseline_duration_s}
            except (RuntimeError,ValueError,AssertionError) as error:
                record={"seed":seed,"route":method,"status":"planning_failure",
                        "reason":str(error)}
            print("PREFLIGHT",json.dumps(record),flush=True)
            results.append(record)
    summary={"scope":"posthoc development route-score diagnostics, no native flight or independent holdout",
             "expected":len(SEEDS)*len(METHODS),"scenarios":results,
             "accepted_detours":sum(x["status"]=="map_oracle_detour_candidate" for x in results),
             "planning_failures":sum(x["status"]=="planning_failure" for x in results)}
    out=Path("results/phase17b_preflight");out.mkdir(parents=True,exist_ok=True)
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
if __name__=="__main__":main()
