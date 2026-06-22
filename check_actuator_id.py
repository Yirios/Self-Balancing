"""Validate one actuator-identification CSV or all CSV files in a directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def as_int(row: dict[str, str], name: str) -> int:
    return int(row[name])


def expected_drive_registers(side: str, pwm: int) -> tuple[int, int]:
    if side == "L":
        return (7200, 7200 - pwm) if pwm > 0 else (7200 + pwm, 7200)
    return (7200 - pwm, 7200) if pwm > 0 else (7200, 7200 + pwm)


def check_file(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    errors: list[str] = []
    warnings: list[str] = []
    if len(rows) < 2:
        return {"file": str(path), "valid": False, "errors": ["样本少于 2"], "warnings": []}

    times = [as_int(row, "time_us") for row in rows]
    ids = [as_int(row, "sample_id") for row in rows]
    periods = [b - a for a, b in zip(times, times[1:])]
    id_steps = [b - a for a, b in zip(ids, ids[1:])]
    if any(period <= 0 for period in periods):
        errors.append("time_us 非严格递增")
    if ids[0] != 0:
        errors.append(f"首个 sample_id 应为 0，实际为 {ids[0]}")
    if any(step != 1 for step in id_steps):
        errors.append(f"sample_id 不连续，缺样估计 {sum(max(step - 1, 0) for step in id_steps)}")

    median_period = statistics.median(periods)
    jitter = [abs(period - median_period) for period in periods]
    jitter_p99 = percentile(jitter, 0.99)
    if jitter_p99 > median_period * 0.20:
        warnings.append("99% 周期抖动超过目标周期的 20%")

    register_mismatches = 0
    for row in rows:
        for side in ("L", "R"):
            mode = row[f"bridge_mode_{side}"]
            applied = as_int(row, f"pwm_applied_{side}")
            command = as_int(row, f"pwm_cmd_{side}")
            if mode == "DRIVE":
                expected = expected_drive_registers(side, applied)
                actual = (as_int(row, f"pwm_reg_{side}1"), as_int(row, f"pwm_reg_{side}2"))
                if command != applied or actual != expected:
                    register_mismatches += 1
            elif applied != 0:
                register_mismatches += 1
    if register_mismatches:
        errors.append(f"PWM/桥寄存器不一致 {register_mismatches} 处")

    fault_rows = [row for row in rows if as_int(row, "fault_flags") != 0]
    if fault_rows:
        errors.append(f"检测到 fault_flags，影响 {len(fault_rows)} 个样本")
    states = {row["state"] for row in rows}
    events = {as_int(row, "event_code") for row in rows}
    if "COAST" not in states or 3 not in events:
        errors.append("未检测到带 event_code=3 的 COAST 转换")
    if "COMPLETE" not in states or 5 not in events:
        errors.append("run 未正常 COMPLETE")

    max_speed_l = max(abs(float(row["wheel_speed_L_rad_s"] or 0.0)) for row in rows)
    max_speed_r = max(abs(float(row["wheel_speed_R_rad_s"] or 0.0)) for row in rows)
    voltages = [float(row["battery_voltage_v"]) for row in rows]
    max_delta = max(
        max(abs(as_int(row, "encoder_delta_L")), abs(as_int(row, "encoder_delta_R")))
        for row in rows
    )
    if max_delta > 20_000:
        errors.append("编码器单样本跳变超过 20000 count")

    return {
        "file": str(path),
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "samples": len(rows),
        "duration_s": (times[-1] - times[0]) / 1e6,
        "sample_rate_median_hz": 1e6 / median_period,
        "period_jitter_p99_us": jitter_p99,
        "max_wheel_speed_rad_s": {"L": max_speed_l, "R": max_speed_r},
        "battery_voltage_v": {"min": min(voltages), "max": max(voltages)},
        "max_abs_encoder_delta": max_delta,
        "register_mismatches": register_mismatches,
    }


def matrix_summary(paths: list[Path], results: list[dict[str, object]]) -> dict[str, object]:
    groups: dict[str, dict[str, object]] = {}
    unclassified: list[str] = []
    for path, result in zip(paths, results):
        metadata_path = path.with_suffix(".json")
        if not metadata_path.exists():
            unclassified.append(str(path))
            continue
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            experiment = metadata["experiment"]
            run_config = metadata["run_config"]
            stimulus = (
                run_config.get("target_speed_rad_s") if experiment == "coast"
                else [run_config.get("pwm_a"), run_config.get("pwm_b")] if experiment == "pwmstep"
                else run_config.get("pwm_a")
            )
            key_data = {
                "experiment": experiment,
                "side": metadata["side"],
                "direction": metadata["direction"],
                "inertia": metadata.get("wheel_inertia_config"),
                "stimulus": stimulus,
            }
        except (KeyError, TypeError, json.JSONDecodeError):
            unclassified.append(str(path))
            continue
        key = json.dumps(key_data, ensure_ascii=False, sort_keys=True)
        expected = 10 if experiment == "pwmstep" else 3 if (
            experiment == "coast" and key_data["inertia"] == "J2"
        ) else 5
        group = groups.setdefault(key, {"combination": key_data, "expected": expected, "runs": 0, "valid_runs": 0})
        group["runs"] += 1
        group["valid_runs"] += int(bool(result["valid"]))
    combinations = list(groups.values())
    return {
        "complete": bool(combinations) and all(item["valid_runs"] >= item["expected"] for item in combinations),
        "combinations": combinations,
        "unclassified": unclassified,
        "note": "仅统计已出现的组合；尚未采集、因而没有 JSON 的组合需对照实验规范另行确认。",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.path.exists():
        parser.error(f"路径不存在: {args.path}")
    paths = sorted(args.path.glob("*.csv")) if args.path.is_dir() else [args.path]
    if not paths:
        parser.error(f"目录内没有 CSV: {args.path}")
    results = [check_file(path) for path in paths]
    report: dict[str, object] = {"valid": bool(results) and all(item["valid"] for item in results), "runs": results}
    if args.path.is_dir():
        matrix = matrix_summary(paths, results)
        report["matrix"] = matrix
        report["valid"] = bool(report["valid"] and matrix["complete"] and not matrix["unclassified"])
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
