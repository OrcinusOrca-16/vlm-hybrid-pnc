"""Fast cached development-only refinement, soft-margin ablation and pruning."""

import json
from pathlib import Path
from itertools import product
from collections import Counter
import numpy as np
from scripts.planning.development.run_sampling_development_study import OUT
from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios


def main():
    pools = {}
    for s in build_development_tuning_scenarios():
        rows = [
            r for r in json.loads((OUT / (s.name + ".json")).read_text()) if all(r["flags"]) and r["params"][3] == 0.0
        ]
        pools[s.name] = rows
    lateral = {
        "d05": tuple(i / 2 for i in range(-4, 5)),
        "d04": tuple(i * 0.4 for i in range(-5, 6)),
        "nonuniform": (-2.0, -1.6, -1.5, -1.2, -1.0, -0.8, -0.4, 0.0, 0.4, 0.8, 1.0, 1.2, 1.5, 1.6, 2.0),
        "compact": (-2.0, -1.6, -1.5, -1.0, -0.8, -0.4, 0.0, 0.4, 0.8, 1.0, 1.5, 1.6, 2.0),
        "d025": tuple(i / 4 for i in range(-8, 9)),
    }
    lengths = {
        "base": (6.0, 10.0, 14.0, 18.0, 22.0),
        "balanced": (6.0, 8.0, 14.0, 18.0, 22.0),
        "compact": (6.0, 8.0, 14.0, 22.0),
        "reduced": (8.0, 14.0, 22.0),
        "dense": (6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0),
        "long": (8.0, 14.0, 18.0, 22.0, 24.0, 26.0),
    }
    slopes = {
        "strict": (0.0,),
        "s01": (-0.1, 0.0, 0.1),
        "s015": (-0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15),
        "sparse": (-0.15, -0.1, 0.0, 0.1, 0.15),
        "wide": (-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3),
        "fine": tuple(i * 0.025 for i in range(-8, 9)),
    }
    results = []
    for (ln, lat), (bn, lens), (jn, sl) in product(lateral.items(), lengths.items(), slopes.items()):
        lat = tuple(round(x, 6) for x in lat)
        sl = tuple(round(x, 6) for x in sl)
        eligible = {
            n: [r for r in rows if r["params"][0] in lat and r["params"][1] in lens and r["params"][2] in sl]
            for n, rows in pools.items()
        }
        for w, rw in product((0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0, 15.0, 25.0), (0.0, 0.1, 0.25, 0.5, 1.0)):
            selected = {}
            for n, rows in eligible.items():
                if rows:
                    selected[n] = min(
                        rows, key=lambda r: r["geo"] + w * r["deficit"] + rw * (0.5 / r["clearance"]) ** 2
                    )
            if not selected:
                continue
            vals = list(selected.values())
            results.append(
                dict(
                    name=f"{ln}/{bn}/{jn}/w{w}/r{rw}",
                    lat=lat,
                    lengths=lens,
                    slopes=sl,
                    weight=w,
                    robust_weight=rw,
                    count=len(lat) * len(lens) * len(sl),
                    success=len(vals),
                    failures=[n for n in pools if n not in selected],
                    mean_speed=float(np.mean([r["speed"] for r in vals])),
                    min_speed=min(r["speed"] for r in vals),
                    early_speed=(
                        selected["tuning_obstacle_longitudinal_18.00"]["speed"]
                        if "tuning_obstacle_longitudinal_18.00" in selected
                        else None
                    ),
                    geo=float(np.mean([r["geo"] for r in vals])),
                    deficit=float(np.mean([r["deficit"] for r in vals])),
                    clearance=min(r["clearance"] for r in vals),
                    curvature_margin=min(r["curvature_margin"] for r in vals),
                    steering_margin=min(r["steering_margin"] for r in vals),
                    selected=selected,
                )
            )
    results.sort(key=lambda r: (-r["success"], r["deficit"], r["geo"], r["count"]))
    (OUT / "targeted.json").write_text(json.dumps(results))
    # Print a compact table at each candidate budget, without selecting by a
    # synthetic aggregate score. Common-success comparisons are reported later.
    for budget in (50, 100, 150, 200, 300, 500, 1000, 5000):
        a = [r for r in results if r["count"] <= budget]
        best = a[0] if a else None
        if best:
            print("budget", budget, {k: v for k, v in best.items() if k not in ("selected",)}, flush=True)
    for label, condition in [
        ("margin", lambda r: r["robust_weight"] == 0.5),
        ("no_margin", lambda r: r["robust_weight"] == 0.0),
    ]:
        a = [r for r in results if r["count"] <= 300 and condition(r)]
        if a:
            print(label, {k: v for k, v in a[0].items() if k != "selected"}, flush=True)


if __name__ == "__main__":
    main()
