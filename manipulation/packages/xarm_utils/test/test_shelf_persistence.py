"""Invalid proposals must not overwrite the working arena calibration."""

import json

import pytest

from xarm_utils.shelf_levels import (
    ShelfCalibrationError,
    save_detected_levels,
    write_json_atomic,
)


def test_invalid_levels_leave_existing_calibration_unchanged(tmp_path):
    path = tmp_path / "levels.json"
    original = (
        '{"frame":"base_link","compartment_height":0.26,"arenas":{"1":[0.4,0.7]}}\n'
    )
    path.write_text(original)
    with pytest.raises(ShelfCalibrationError):
        save_detected_levels(1, [0.7, 0.4], path)
    assert path.read_text() == original
    assert sorted(p.name for p in tmp_path.iterdir()) == ["levels.json"]


def test_valid_levels_preserve_other_arena_and_permissions(tmp_path):
    path = tmp_path / "levels.json"
    path.write_text(
        json.dumps(
            dict(frame="base_link", compartment_height=0.3, arenas={"2": [0.2, 0.6]})
        )
    )
    path.chmod(0o640)
    save_detected_levels(1, [0.4, 0.8], path)
    data = json.loads(path.read_text())
    assert data["arenas"] == {"1": [0.4, 0.8], "2": [0.2, 0.6]}
    assert data["compartment_height"] == 0.3
    assert path.stat().st_mode & 0o777 == 0o640


def test_nonfinite_report_cannot_replace_existing_file(tmp_path):
    path = tmp_path / "geometry.json"
    path.write_text("{}\n")
    with pytest.raises(ValueError):
        write_json_atomic(path, {"height": float("nan")})
    assert path.read_text() == "{}\n"
