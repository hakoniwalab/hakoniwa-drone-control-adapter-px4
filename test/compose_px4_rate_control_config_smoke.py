#!/usr/bin/env python3

import json
import subprocess
import tempfile
from pathlib import Path


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    hakoniwa_txt = repo_root.parent.parent / "config" / "controller" / "param-api-mixer-mujoco.txt"
    extra_json = repo_root / "config" / "px4-controller-extra.sample.json"
    tool = repo_root / "tools" / "compose_px4_rate_control_config.py"

    with tempfile.TemporaryDirectory() as tmpdir:
        output = Path(tmpdir) / "px4-controller-config.json"
        subprocess.run(
            [
                "python3",
                str(tool),
                "--hakoniwa-txt",
                str(hakoniwa_txt),
                "--px4-extra-json",
                str(extra_json),
                "--output",
                str(output),
            ],
            check=True,
        )

        data = json.loads(output.read_text())

        assert data["schema_version"] == 1
        assert "px4_version" in data
        assert "runtime" in data
        assert "attitude_control" in data
        assert "rate_control" in data
        assert "parameters" in data["attitude_control"]
        assert "parameters" in data["rate_control"]

        attitude_params = data["attitude_control"]["parameters"]
        params = data["rate_control"]["parameters"]
        assert abs(attitude_params["MC_ROLL_P"] - 2.5) < 1e-9
        assert abs(attitude_params["MC_PITCH_P"] - 2.5) < 1e-9
        assert abs(attitude_params["MC_YAW_P"] - 0.1) < 1e-9
        assert abs(attitude_params["MC_ROLLRATE_MAX"] - 188.49555921538757) < 1e-9
        assert abs(attitude_params["MC_YAWRATE_MAX"] - 18.84955592153876) < 1e-9
        assert abs(attitude_params["MC_YAW_WEIGHT"] - 0.4) < 1e-9
        assert abs(params["MC_ROLLRATE_P"] - 1.5) < 1e-9
        assert abs(params["MC_PITCHRATE_D"] - 0.02) < 1e-9
        assert abs(params["MC_YAWRATE_P"] - 0.452) < 1e-9
        assert abs(params["MC_RR_INT_LIM"] - 0.3) < 1e-9
        assert abs(data["runtime"]["attitude_hz"] - 1000.0) < 1e-9
        assert abs(data["runtime"]["rate_hz"] - 1000.0) < 1e-9

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
