from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import median
from typing import Iterable

from core.drone_params import DroneParams, get_drone_params


NUMERIC_FIELDS = {
    "mass",
    "ixx",
    "iyy",
    "izz",
    "arm_length",
    "k_drag",
    "j_rotor",
    "kf",
    "km",
    "tau_motor",
    "omega_max",
    "bem_radius",
    "bem_chord",
    "bem_num_blades",
    "bem_theta_tip",
    "bem_theta_root",
}


def estimate_drone_params_from_csv(
    csv_path: str | Path,
    *,
    base_profile: str = "default_1kg",
    output_name: str = "calibrated_candidate",
) -> DroneParams:
    """Build a candidate DroneParams profile from CSV median columns.

    This is an interface-level calibration helper: it aggregates already
    estimated physical columns from flight-log or bench-test preprocessing. It
    does not claim to identify parameters directly from raw telemetry.
    """
    rows = _read_rows(csv_path)
    base = get_drone_params(base_profile)
    values = _median_numeric_values(rows)

    inertia = (
        values.get("ixx", base.inertia[0]),
        values.get("iyy", base.inertia[1]),
        values.get("izz", base.inertia[2]),
    )
    rotor_model = _first_string(rows, "rotor_model") or base.rotor_model
    notes = (
        f"Candidate profile estimated from {Path(csv_path).name}; "
        "requires validation against independent flight data."
    )
    return DroneParams(
        name=output_name,
        mass=values.get("mass", base.mass),
        inertia=inertia,
        arm_length=values.get("arm_length", base.arm_length),
        k_drag=values.get("k_drag", base.k_drag),
        j_rotor=values.get("j_rotor", base.j_rotor),
        kf=values.get("kf", base.kf),
        km=values.get("km", base.km),
        tau_motor=values.get("tau_motor", base.tau_motor),
        omega_max=values.get("omega_max", base.omega_max),
        rotor_model=rotor_model,
        bem_radius=values.get("bem_radius", base.bem_radius),
        bem_chord=values.get("bem_chord", base.bem_chord),
        bem_num_blades=int(round(values.get("bem_num_blades", float(base.bem_num_blades)))),
        bem_theta_tip=values.get("bem_theta_tip", base.bem_theta_tip),
        bem_theta_root=values.get("bem_theta_root", base.bem_theta_root),
        notes=notes,
    )


def write_candidate_profile(params: DroneParams, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(params.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
    return target


def _read_rows(csv_path: str | Path) -> list[dict[str, str]]:
    with Path(csv_path).open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("calibration CSV has no data rows")
    return rows


def _median_numeric_values(rows: Iterable[dict[str, str]]) -> dict[str, float]:
    columns: dict[str, list[float]] = {field: [] for field in NUMERIC_FIELDS}
    for row in rows:
        for field in NUMERIC_FIELDS:
            raw = row.get(field)
            if raw is None or raw == "":
                continue
            try:
                columns[field].append(float(raw))
            except ValueError:
                continue
    return {field: float(median(values)) for field, values in columns.items() if values}


def _first_string(rows: Iterable[dict[str, str]], field: str) -> str | None:
    for row in rows:
        value = (row.get(field) or "").strip()
        if value:
            return value
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a candidate DroneParams profile JSON from calibration CSV columns.")
    parser.add_argument("csv_path")
    parser.add_argument("--base-profile", default="default_1kg")
    parser.add_argument("--name", default="calibrated_candidate")
    parser.add_argument("--output", default="outputs/calibration/drone_params_candidate.json")
    args = parser.parse_args()

    params = estimate_drone_params_from_csv(
        args.csv_path,
        base_profile=args.base_profile,
        output_name=args.name,
    )
    out = write_candidate_profile(params, args.output)
    print(f"wrote {out}")
    print(f"hover_omega = {params.hover_omega:.3f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

