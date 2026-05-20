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
        assert "common" in data
        assert "position_control" in data
        assert "attitude_control" in data
        assert "control_allocation" in data
        assert "rate_control" in data
        assert "parameters" in data["position_control"]
        assert "parameters" in data["attitude_control"]
        assert "parameters" in data["control_allocation"]
        assert "parameters" in data["rate_control"]

        common_params = data["common"]["parameters"]
        position_params = data["position_control"]["parameters"]
        attitude_params = data["attitude_control"]["parameters"]
        control_allocation_params = data["control_allocation"]["parameters"]
        params = data["rate_control"]["parameters"]
        assert abs(position_params["MPC_Z_P"] - 10.0) < 1e-9
        assert abs(position_params["MPC_Z_VEL_P_ACC"] - 15.0) < 1e-9
        assert abs(position_params["MPC_Z_VEL_D_ACC"] - 10.0) < 1e-9
        assert abs(position_params["MPC_Z_VEL_MAX_UP"] - 10.0) < 1e-9
        assert abs(position_params["MPC_Z_VEL_MAX_DN"] - 10.0) < 1e-9
        assert abs(common_params["MPC_THR_HOVER"] - 0.5) < 1e-9
        assert abs(common_params["MPC_THR_MAX"] - 0.9) < 1e-9
        assert abs(common_params["MPC_THR_MIN"] - 0.1) < 1e-9
        assert "MPC_THR_HOVER" not in position_params
        assert "MPC_THR_MAX" not in position_params
        assert "MPC_THR_MIN" not in position_params
        assert abs(attitude_params["MC_ROLL_P"] - 2.5) < 1e-9
        assert abs(attitude_params["MC_PITCH_P"] - 2.5) < 1e-9
        assert abs(attitude_params["MC_YAW_P"] - 0.1) < 1e-9
        assert abs(attitude_params["MC_ROLLRATE_MAX"] - 188.49555921538757) < 1e-9
        assert abs(attitude_params["MC_YAWRATE_MAX"] - 18.84955592153876) < 1e-9
        assert abs(attitude_params["MC_YAW_WEIGHT"] - 0.4) < 1e-9
        assert abs(control_allocation_params["CA_RPY_NORMALIZE"] - 1.0) < 1e-9
        assert abs(control_allocation_params["CA_METRIC_ALLOCATION"] - 0.0) < 1e-9
        assert abs(control_allocation_params["CA_UPDATE_NORMALIZATION_SCALE"] - 1.0) < 1e-9
        assert abs(position_params["MPC_XY_P"] - 6.0) < 1e-9
        assert abs(position_params["MPC_XY_VEL_P_ACC"] - 10.0) < 1e-9
        assert abs(position_params["MPC_XY_VEL_D_ACC"] - 0.10) < 1e-9
        assert abs(position_params["MPC_XY_VEL_MAX"] - 20.0) < 1e-9
        assert abs(position_params["MPC_TILTMAX_AIR"] - 0.2617993877991494) < 1e-9
        assert abs(position_params["MPC_THR_XY_MARG"] - 0.3) < 1e-9
        assert abs(params["MC_ROLLRATE_P"] - 1.5) < 1e-9
        assert abs(params["MC_PITCHRATE_D"] - 0.02) < 1e-9
        assert abs(params["MC_YAWRATE_P"] - 0.452) < 1e-9
        assert abs(params["MC_RR_INT_LIM"] - 0.3) < 1e-9
        assert abs(data["runtime"]["altitude_hz"] - 1000.0) < 1e-9
        assert abs(data["runtime"]["attitude_hz"] - 1000.0) < 1e-9
        assert abs(data["runtime"]["horizontal_hz"] - 1000.0) < 1e-9
        assert abs(data["runtime"]["rate_hz"] - 1000.0) < 1e-9

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
