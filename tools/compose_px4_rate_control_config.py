#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


PX4_GIT_COMMIT = "a1726d316a941af9524f6279eb293a713d8fdcac"
PX4_GIT_DESCRIBE = "v1.17.0-alpha1-1702-ga1726d316a"


def parse_hakoniwa_txt(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}

    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        parts = line.split()

        if len(parts) != 2:
            raise ValueError(f"invalid parameter line at {path}:{lineno}: {raw_line}")

        key, value = parts
        values[key] = float(value)

    return values


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def require(params: dict[str, float], key: str) -> float:
    if key not in params:
        raise KeyError(f"missing required Hakoniwa parameter: {key}")
    return float(params[key])


def get_optional(params: dict[str, float], key: str, default: float) -> float:
    return float(params.get(key, default))


def require_same_value(params: dict[str, float], key_a: str, key_b: str, tolerance: float = 1e-9) -> float:
    a = require(params, key_a)
    b = require(params, key_b)
    if abs(a - b) > tolerance:
        raise ValueError(f"expected matching Hakoniwa parameters for PX4 shared axis mapping: {key_a}={a}, {key_b}={b}")
    return a


def compose_rate_control_parameters(
    hakoniwa_params: dict[str, float],
    px4_extra: dict[str, float],
    expand_gain_k: bool,
    allow_torque_limit_heuristic: bool,
) -> dict[str, float]:
    parameters = {
        "MC_ROLLRATE_P": require(hakoniwa_params, "PID_ROLL_RATE_Kp"),
        "MC_ROLLRATE_I": require(hakoniwa_params, "PID_ROLL_RATE_Ki"),
        "MC_ROLLRATE_D": require(hakoniwa_params, "PID_ROLL_RATE_Kd"),
        "MC_PITCHRATE_P": require(hakoniwa_params, "PID_PITCH_RATE_Kp"),
        "MC_PITCHRATE_I": require(hakoniwa_params, "PID_PITCH_RATE_Ki"),
        "MC_PITCHRATE_D": require(hakoniwa_params, "PID_PITCH_RATE_Kd"),
        "MC_YAWRATE_P": require(hakoniwa_params, "PID_YAW_RATE_Kp"),
        "MC_YAWRATE_I": require(hakoniwa_params, "PID_YAW_RATE_Ki"),
        "MC_YAWRATE_D": require(hakoniwa_params, "PID_YAW_RATE_Kd"),
        "MC_ROLLRATE_FF": get_optional(px4_extra, "MC_ROLLRATE_FF", 0.0),
        "MC_PITCHRATE_FF": get_optional(px4_extra, "MC_PITCHRATE_FF", 0.0),
        "MC_YAWRATE_FF": get_optional(px4_extra, "MC_YAWRATE_FF", 0.0),
    }

    if expand_gain_k:
        parameters["MC_ROLLRATE_P"] *= get_optional(px4_extra, "MC_ROLLRATE_K", 1.0)
        parameters["MC_ROLLRATE_I"] *= get_optional(px4_extra, "MC_ROLLRATE_K", 1.0)
        parameters["MC_ROLLRATE_D"] *= get_optional(px4_extra, "MC_ROLLRATE_K", 1.0)
        parameters["MC_ROLLRATE_FF"] *= get_optional(px4_extra, "MC_ROLLRATE_K", 1.0)

        parameters["MC_PITCHRATE_P"] *= get_optional(px4_extra, "MC_PITCHRATE_K", 1.0)
        parameters["MC_PITCHRATE_I"] *= get_optional(px4_extra, "MC_PITCHRATE_K", 1.0)
        parameters["MC_PITCHRATE_D"] *= get_optional(px4_extra, "MC_PITCHRATE_K", 1.0)
        parameters["MC_PITCHRATE_FF"] *= get_optional(px4_extra, "MC_PITCHRATE_K", 1.0)

        parameters["MC_YAWRATE_P"] *= get_optional(px4_extra, "MC_YAWRATE_K", 1.0)
        parameters["MC_YAWRATE_I"] *= get_optional(px4_extra, "MC_YAWRATE_K", 1.0)
        parameters["MC_YAWRATE_D"] *= get_optional(px4_extra, "MC_YAWRATE_K", 1.0)
        parameters["MC_YAWRATE_FF"] *= get_optional(px4_extra, "MC_YAWRATE_K", 1.0)

    if "MC_RR_INT_LIM" in px4_extra:
        parameters["MC_RR_INT_LIM"] = float(px4_extra["MC_RR_INT_LIM"])
    elif allow_torque_limit_heuristic and "PID_ROLL_TORQUE_MAX" in hakoniwa_params:
        parameters["MC_RR_INT_LIM"] = float(hakoniwa_params["PID_ROLL_TORQUE_MAX"])

    if "MC_PR_INT_LIM" in px4_extra:
        parameters["MC_PR_INT_LIM"] = float(px4_extra["MC_PR_INT_LIM"])
    elif allow_torque_limit_heuristic and "PID_PITCH_TORQUE_MAX" in hakoniwa_params:
        parameters["MC_PR_INT_LIM"] = float(hakoniwa_params["PID_PITCH_TORQUE_MAX"])

    if "MC_YR_INT_LIM" in px4_extra:
        parameters["MC_YR_INT_LIM"] = float(px4_extra["MC_YR_INT_LIM"])
    elif allow_torque_limit_heuristic and "PID_YAW_TORQUE_MAX" in hakoniwa_params:
        parameters["MC_YR_INT_LIM"] = float(hakoniwa_params["PID_YAW_TORQUE_MAX"])

    return parameters


def compose_attitude_control_parameters(
    hakoniwa_params: dict[str, float],
    px4_extra: dict[str, float],
) -> dict[str, float]:
    return {
        "MC_ROLL_P": require(hakoniwa_params, "PID_ROLL_Kp"),
        "MC_PITCH_P": require(hakoniwa_params, "PID_PITCH_Kp"),
        "MC_YAW_P": require(hakoniwa_params, "PID_YAW_Kp"),
        "MC_ROLLRATE_MAX": require(hakoniwa_params, "PID_ROLL_RPM_MAX") * 2.0 * 3.141592653589793 / 60.0,
        "MC_PITCHRATE_MAX": require(hakoniwa_params, "PID_PITCH_RPM_MAX") * 2.0 * 3.141592653589793 / 60.0,
        "MC_YAWRATE_MAX": require(hakoniwa_params, "PID_YAW_RPM_MAX") * 2.0 * 3.141592653589793 / 60.0,
        "MC_YAW_WEIGHT": get_optional(px4_extra, "MC_YAW_WEIGHT", 0.4),
    }


def compose_altitude_control_parameters(
    hakoniwa_params: dict[str, float],
    px4_extra: dict[str, float],
) -> dict[str, float]:
    max_spd = require(hakoniwa_params, "PID_ALT_MAX_SPD")
    return {
        "MPC_Z_P": require(hakoniwa_params, "PID_ALT_Kp"),
        "MPC_Z_VEL_P_ACC": require(hakoniwa_params, "PID_ALT_SPD_Kp"),
        "MPC_Z_VEL_I_ACC": require(hakoniwa_params, "PID_ALT_SPD_Ki"),
        "MPC_Z_VEL_D_ACC": require(hakoniwa_params, "PID_ALT_SPD_Kd"),
        "MPC_Z_VEL_MAX_UP": max_spd,
        "MPC_Z_VEL_MAX_DN": max_spd,
        "MPC_THR_HOVER": get_optional(px4_extra, "MPC_THR_HOVER", 0.5),
        "MPC_THR_MIN": get_optional(px4_extra, "MPC_THR_MIN", 0.1),
        "MPC_THR_MAX": get_optional(px4_extra, "MPC_THR_MAX", 0.9),
    }


def compose_horizontal_control_parameters(
    hakoniwa_params: dict[str, float],
    px4_extra: dict[str, float],
) -> dict[str, float]:
    tilt_limit_deg = min(
        require(hakoniwa_params, "PID_POS_MAX_ROLL"),
        require(hakoniwa_params, "PID_POS_MAX_PITCH"),
    )
    return {
        "MPC_XY_P": require_same_value(hakoniwa_params, "PID_POS_X_Kp", "PID_POS_Y_Kp"),
        "MPC_XY_VEL_P_ACC": require_same_value(hakoniwa_params, "PID_POS_VX_Kp", "PID_POS_VY_Kp"),
        "MPC_XY_VEL_I_ACC": require_same_value(hakoniwa_params, "PID_POS_VX_Ki", "PID_POS_VY_Ki"),
        "MPC_XY_VEL_D_ACC": require_same_value(hakoniwa_params, "PID_POS_VX_Kd", "PID_POS_VY_Kd"),
        "MPC_XY_VEL_MAX": require(hakoniwa_params, "PID_POS_MAX_SPD"),
        "MPC_TILTMAX_AIR": tilt_limit_deg * 3.141592653589793 / 180.0,
        "MPC_THR_XY_MARG": get_optional(px4_extra, "MPC_THR_XY_MARG", 0.3),
        "MPC_ACC_DECOUPLE": get_optional(px4_extra, "MPC_ACC_DECOUPLE", 1.0),
        "MPC_THR_HOVER": get_optional(px4_extra, "MPC_THR_HOVER", 0.5),
        "MPC_THR_MIN": get_optional(px4_extra, "MPC_THR_MIN", 0.1),
        "MPC_THR_MAX": get_optional(px4_extra, "MPC_THR_MAX", 0.9),
    }


def compose_control_allocation_parameters(
    hakoniwa_params: dict[str, float],
    px4_extra: dict[str, float],
) -> dict[str, float]:
    return {
        "CA_RPY_NORMALIZE": get_optional(px4_extra, "CA_RPY_NORMALIZE", 1.0),
        "CA_METRIC_ALLOCATION": get_optional(px4_extra, "CA_METRIC_ALLOCATION", 0.0),
        "CA_UPDATE_NORMALIZATION_SCALE": get_optional(px4_extra, "CA_UPDATE_NORMALIZATION_SCALE", 1.0),
        "CA_HOVER_DUTY": get_optional(px4_extra, "CA_HOVER_DUTY", 0.120311),
        "MASS": require(hakoniwa_params, "MASS"),
        "GRAVITY": require(hakoniwa_params, "GRAVITY"),
    }


def compose_runtime(hakoniwa_params: dict[str, float]) -> dict[str, float]:
    def derive_frequency(cycle_key: str) -> float:
        cycle = get_optional(hakoniwa_params, cycle_key, 0.0)
        sim_dt = get_optional(hakoniwa_params, "SIMULATION_DELTA_TIME", 0.0)

        if cycle > 0.0:
            return 1.0 / cycle
        if sim_dt > 0.0:
            return 1.0 / sim_dt
        raise ValueError(f"failed to derive frequency from {cycle_key} or SIMULATION_DELTA_TIME")

    return {
        "altitude_hz": derive_frequency("PID_ALT_CONTROL_CYCLE"),
        "attitude_hz": derive_frequency("ANGULAR_CONTROL_CYCLE"),
        "horizontal_hz": derive_frequency("SPD_CONTROL_CYCLE"),
        "rate_hz": derive_frequency("ANGULAR_RATE_CONTROL_CYCLE")
    }


def compose_config(
    hakoniwa_txt_path: Path,
    px4_extra_path: Path,
    output_path: Path,
    expand_gain_k: bool,
    allow_torque_limit_heuristic: bool,
) -> None:
    hakoniwa_params = parse_hakoniwa_txt(hakoniwa_txt_path)
    px4_extra_json = load_json(px4_extra_path)
    altitude_extra = px4_extra_json.get("altitude_control", {}).get("extra", {})
    attitude_extra = px4_extra_json.get("attitude_control", {}).get("extra", {})
    control_allocation_extra = px4_extra_json.get("control_allocation", {}).get("extra", {})
    horizontal_extra = px4_extra_json.get("horizontal_control", {}).get("extra", {})
    px4_extra = px4_extra_json.get("rate_control", {}).get("extra", {})

    config = {
        "schema_version": 1,
        "px4_version": {
            "git_commit": PX4_GIT_COMMIT,
            "git_describe": PX4_GIT_DESCRIBE,
        },
        "runtime": compose_runtime(hakoniwa_params),
        "altitude_control": {
            "parameters": compose_altitude_control_parameters(
                hakoniwa_params,
                altitude_extra,
            )
        },
        "attitude_control": {
            "parameters": compose_attitude_control_parameters(
                hakoniwa_params,
                attitude_extra,
            )
        },
        "control_allocation": {
            "parameters": compose_control_allocation_parameters(
                hakoniwa_params,
                control_allocation_extra,
            )
        },
        "horizontal_control": {
            "parameters": compose_horizontal_control_parameters(
                hakoniwa_params,
                horizontal_extra,
            )
        },
        "rate_control": {
            "parameters": compose_rate_control_parameters(
                hakoniwa_params,
                px4_extra,
                expand_gain_k=expand_gain_k,
                allow_torque_limit_heuristic=allow_torque_limit_heuristic,
            )
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compose px4-controller-config.json for PX4 control backends from Hakoniwa txt and PX4 extra json."
    )
    parser.add_argument("--hakoniwa-txt", required=True, help="Hakoniwa controller parameter txt")
    parser.add_argument("--px4-extra-json", required=True, help="PX4 extra json")
    parser.add_argument("--output", required=True, help="Output px4-controller-config.json path")
    parser.add_argument("--expand-gain-k", action="store_true", help="Expand MC_*RATE_K into P/I/D/FF")
    parser.add_argument(
        "--allow-torque-limit-to-integrator-limit-heuristic",
        action="store_true",
        help="Fallback to PID_*_TORQUE_MAX when MC_*_INT_LIM is absent",
    )
    args = parser.parse_args()

    compose_config(
        hakoniwa_txt_path=Path(args.hakoniwa_txt),
        px4_extra_path=Path(args.px4_extra_json),
        output_path=Path(args.output),
        expand_gain_k=args.expand_gain_k,
        allow_torque_limit_heuristic=args.allow_torque_limit_to_integrator_limit_heuristic,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
