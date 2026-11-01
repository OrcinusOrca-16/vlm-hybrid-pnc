"""Check sampled winners and late-case diagnostic candidates on a finer mesh."""

import json
from dataclasses import replace
from scripts.planning.development.run_sampling_development_study import OUT, scenarios, measure


def main():
    ss = scenarios()
    results = {}
    for s in ss:
        if s.name != "obstacle_longitudinal_36.00":
            continue
        rows = json.loads((OUT / (s.name + ".json")).read_text())
        rows = [r for r in rows if all(r["flags"])]
        checked = []
        fine = replace(s, resolution_m=0.05)
        object.__setattr__(fine, "_study_initial", s._study_initial)
        for r in rows:
            f = measure(fine, tuple(r["params"]))
            checked.append(dict(coarse=r, fine=f))
        good = [r for r in checked if all(r["fine"]["flags"])]
        results[s.name] = dict(
            coarse_feasible=len(rows),
            fine_feasible=len(good),
            best_speed=max(good, key=lambda r: r["fine"]["speed"]) if good else None,
            best_clearance=max(good, key=lambda r: r["fine"]["clearance"]) if good else None,
            rows=checked,
        )
        print({k: v for k, v in results[s.name].items() if k != "rows"}, flush=True)
    targeted = json.loads((OUT / "targeted.json").read_text())
    selected_checks = {}
    names = (
        "d05/balanced/strict/w7.5/r0.0",
        "d05/balanced/s01/w7.5/r0.0",
        "d025/balanced/strict/w7.5/r0.0",
        "d05/balanced/strict/w4.0/r0.0",
    )
    for config in targeted:
        if config["name"] not in names:
            continue
        checks = {}
        for s in ss:
            if s.name not in config["selected"]:
                continue
            fine = replace(s, resolution_m=0.05)
            object.__setattr__(fine, "_study_initial", s._study_initial)
            checks[s.name] = measure(fine, tuple(config["selected"][s.name]["params"]))
        selected_checks[config["name"]] = checks
        print(
            config["name"],
            "fine success",
            sum(all(r["flags"]) for r in checks.values()),
            "fine min clearance",
            min(r["clearance"] for r in checks.values()),
            "fine min speed",
            min(r.get("speed", float("inf")) for r in checks.values()),
            flush=True,
        )
    results["selected_checks"] = selected_checks
    (OUT / "dense_validation.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
