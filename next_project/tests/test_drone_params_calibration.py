from __future__ import annotations

import json
from pathlib import Path

from experiments.calibration.drone_params_calibration import (
    estimate_drone_params_from_csv,
    write_candidate_profile,
)


def test_estimate_drone_params_from_csv_uses_median_columns(tmp_path: Path):
    csv_path = tmp_path / "calibration.csv"
    csv_path.write_text(
        "\n".join([
            "mass,ixx,iyy,izz,kf,km,omega_max,rotor_model",
            "1.0,0.01,0.011,0.02,1.0e-5,2.0e-7,1000,simple",
            "1.2,0.012,0.013,0.022,1.2e-5,2.2e-7,1100,simple",
            "1.4,0.014,0.015,0.024,1.4e-5,2.4e-7,1200,simple",
        ]),
        encoding="utf-8",
    )

    params = estimate_drone_params_from_csv(csv_path, output_name="bench_candidate")

    assert params.name == "bench_candidate"
    assert params.mass == 1.2
    assert params.inertia == (0.012, 0.013, 0.022)
    assert params.kf == 1.2e-5
    assert params.km == 2.2e-7
    assert params.omega_max == 1100.0
    assert params.hover_omega < params.omega_max


def test_write_candidate_profile_outputs_json(tmp_path: Path):
    csv_path = tmp_path / "calibration.csv"
    csv_path.write_text(
        "mass,kf,km,omega_max\n1.0,1.0e-5,2.0e-7,1000\n",
        encoding="utf-8",
    )
    params = estimate_drone_params_from_csv(csv_path)
    out = write_candidate_profile(params, tmp_path / "candidate.json")

    payload = json.loads(out.read_text(encoding="utf-8"))

    assert payload["name"] == "calibrated_candidate"
    assert payload["mass"] == 1.0
    assert "hover_omega" in payload

