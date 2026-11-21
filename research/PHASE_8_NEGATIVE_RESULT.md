# Phase 8 stopping-envelope ablation: verified negative result

**Experiment:** GitHub Actions run [44883525144](https://github.com/lalitkumar790/mars-lora/actions/runs/44883525144), native MuJoCo 3.3.2, 21 November 2026. This was an exploratory *development-city* comparison, not independent heldout validation.

The paired experiment used two seeds (91311, 20483), two routes (shortest, RF-aware), heavy UAV, 30-second LiDAR blackout (45–75 s), matching localization seeds/wind, and 240-second simulated maximum. Both methods executed all four cases.

| Outcome | Frozen V5 | Phase 8 added guard |
|---|---:|---:|
| Safety pass (mission completion + 0 contacts + min clearance >=0.25 m) | 1/4 | 1/4 |
| Mission completed | 4/4 | 4/4 |
| Trials with contacts | 2/4 | 2/4 |
| Trials with under-clearance | 2/4 | 2/4 |
| Accepted replans | 7 | 7 |
| Replan failures | 7 | 7 |
| Rejected by new stopping-envelope guard | 0 | 0 |

Contact and under-clearance counts overlap and must not be summed as independent failures. The successful GitHub workflow indicates that tests and experiment **executed**, not that flight safety was achieved.

**Interpretation:** the additional check had no demonstrated impact in this envelope. Passing its isolated unit tests is not a claim about UAV safety. The current pre-existing geometric/low-speed checks may be filtering many dangerous handoffs before the new check is reached, while the remaining flight failures arise from tracking, localization, or contacts unrelated to replanning. Those competing explanations are *hypotheses*, not diagnosed facts yet.

**Next step:** Export continuous physical flight trajectories for each matched trial, plus first MuJoCo contact, first required-clearance violation and worst-clearance pose. Compare the earliest failure timestamp with the actual accepted-replan timeline. Only then revise the controller. The revised instrumentation is a follow-up measurement, not a retroactive change to the completed ablation.

**Stop/go:** reject Phase 8 controller promotion. Preserve the 2D map-aided baseline for reference. No field validation, unknown-map SLAM, or robust LoRa + energy optimization is established.
