"""Configure a WHEELTEC actuator-ID run and save raw CSV + JSON metadata."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SYNC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
FRAME_RECORD = 1
PAYLOAD = struct.Struct("<QIHBBHBBhhhhHHHHiiiiHIQ")
FRAME_SIZE = 5 + PAYLOAD.size + 2
COUNTS_PER_REV = 60_000

STATES = {
    0: "IDLE",
    1: "ARMED",
    2: "SPINUP",
    3: "DRIVE_HOLD",
    4: "COAST",
    5: "BRAKE",
    6: "COMPLETE",
    7: "E_STOP",
}
BRIDGE_MODES = {0: "DRIVE", 1: "COAST", 2: "BRAKE", 3: "OFF"}
EXPERIMENT_CODES = {"coast": 1, "pwmhold": 2, "statictorque": 2, "pwmstep": 3}
SIDE_CODES = {"L": 1, "R": 2}

CSV_FIELDS = [
    "time_us", "sample_id", "experiment_id", "state", "previous_state",
    "event_code", "bridge_mode_L", "bridge_mode_R", "pwm_cmd_L", "pwm_cmd_R",
    "pwm_applied_L", "pwm_applied_R", "pwm_reg_L1", "pwm_reg_L2",
    "pwm_reg_R1", "pwm_reg_R2", "encoder_count_L", "encoder_count_R",
    "encoder_delta_L", "encoder_delta_R", "wheel_angle_L_rad", "wheel_angle_R_rad",
    "wheel_speed_L_rad_s", "wheel_speed_R_rad_s", "battery_voltage_v",
    "fault_flags", "pwm_write_time_us",
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Record:
    values: tuple[int, ...]

    @property
    def state(self) -> int:
        return self.values[3]

    @property
    def event_code(self) -> int:
        return self.values[5]

    def as_row(self, previous_time_us: int | None) -> dict[str, object]:
        (
            time_us, sample_id, experiment_id, state, previous_state, event_code,
            bridge_l, bridge_r, pwm_cmd_l, pwm_cmd_r, pwm_applied_l, pwm_applied_r,
            reg_l1, reg_l2, reg_r1, reg_r2, count_l, count_r, delta_l, delta_r,
            battery_mv, fault_flags, pwm_write_time_us,
        ) = self.values
        dt = (time_us - previous_time_us) * 1e-6 if previous_time_us is not None else math.nan
        speed_l = delta_l * 2.0 * math.pi / COUNTS_PER_REV / dt if dt > 0 else math.nan
        speed_r = delta_r * 2.0 * math.pi / COUNTS_PER_REV / dt if dt > 0 else math.nan
        return dict(zip(CSV_FIELDS, [
            time_us, sample_id, experiment_id, STATES.get(state, state),
            STATES.get(previous_state, previous_state), event_code,
            BRIDGE_MODES.get(bridge_l, bridge_l), BRIDGE_MODES.get(bridge_r, bridge_r),
            pwm_cmd_l, pwm_cmd_r, pwm_applied_l, pwm_applied_r,
            reg_l1, reg_l2, reg_r1, reg_r2, count_l, count_r, delta_l, delta_r,
            f"{count_l * 2.0 * math.pi / COUNTS_PER_REV:.9f}",
            f"{count_r * 2.0 * math.pi / COUNTS_PER_REV:.9f}",
            "" if math.isnan(speed_l) else f"{speed_l:.6f}",
            "" if math.isnan(speed_r) else f"{speed_r:.6f}",
            f"{battery_mv / 1000.0:.3f}", fault_flags, pwm_write_time_us,
        ]))


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.framing_errors = 0

    def feed(self, data: bytes) -> list[Record]:
        self.buffer.extend(data)
        records: list[Record] = []
        while True:
            index = self.buffer.find(SYNC)
            if index < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < 5:
                break
            version, frame_type, payload_size = self.buffer[2:5]
            frame_size = 5 + payload_size + 2
            if version != PROTOCOL_VERSION or frame_type != FRAME_RECORD or payload_size != PAYLOAD.size:
                self.framing_errors += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_size:
                break
            frame = bytes(self.buffer[:frame_size])
            expected_crc = struct.unpack_from("<H", frame, frame_size - 2)[0]
            if crc16_ccitt(frame[:-2]) != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            records.append(Record(PAYLOAD.unpack_from(frame, 5)))
            del self.buffer[:frame_size]
        return records


def auto_port() -> str:
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())
    preferred_words = ("STLink", "ST-Link", "STM32", "Virtual COM", "USB Serial", "CH9102", "CH340")
    candidates = []
    for item in ports:
        description = item.description or ""
        hwid = item.hwid or ""
        device = item.device or ""
        is_usb_path = (
            device.startswith("/dev/serial/by-id/")
            or device.startswith("/dev/ttyACM")
            or device.startswith("/dev/ttyUSB")
            or device.upper().startswith("COM")
        )
        is_named_adapter = any(text in description or text in hwid for text in preferred_words)
        if is_usb_path or is_named_adapter:
            candidates.append(device)
    if not candidates:
        available = ", ".join(item.device for item in ports) or "无"
        raise RuntimeError(
            "未找到可信的 USB/ACM 串口；请用 --serial 显式指定，例如 "
            "/dev/ttyACM0、/dev/ttyUSB0 或 /dev/serial/by-id/...。"
            f" 当前可见串口: {available}"
        )
    return candidates[0]


def default_stem(args: argparse.Namespace) -> str:
    repeat = f"r{args.repeat:02d}"
    if args.experiment == "statictorque":
        return f"statictorque_{args.side}_{args.direction}_{repeat}"
    return f"{args.experiment}_{args.side}_{args.direction}_{args.inertia}_{repeat}"


def git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def command_for(args: argparse.Namespace) -> str:
    target = int(round(abs(args.target_speed) * 1000.0))
    if args.direction == "neg":
        target = -target
    return "IDCFG," + ",".join(map(str, [
        args.experiment_id, EXPERIMENT_CODES[args.experiment], SIDE_CODES[args.side],
        args.pwm_a, args.pwm_b, target, args.armed_ms, args.phase_a_ms,
        args.phase_b_ms, args.coast_ms, int(args.max_speed * 1000),
        int(args.undervoltage * 1000),
    ]))


def require_safety_confirmation(args: argparse.Namespace) -> None:
    missing = []
    if not args.wheels_off_ground:
        missing.append("--wheels-off-ground")
    if args.bridge_driver_part == "UNCONFIRMED":
        missing.append("--bridge-driver-part")
    if args.coast_pin_state == "UNCONFIRMED":
        missing.append("--coast-pin-state")
    if args.brake_pin_state == "UNCONFIRMED":
        missing.append("--brake-pin-state")
    if args.direction == "pos" and args.pwm_a < 0:
        missing.append("正向 run 的 --pwm-a 必须非负")
    if args.direction == "neg" and args.pwm_a > 0:
        missing.append("反向 run 的 --pwm-a 必须非正")
    if max(abs(args.pwm_a), abs(args.pwm_b)) > args.pwm_limit:
        missing.append(f"PWM 超过声明的安全限幅 {args.pwm_limit}")
    if missing and not args.dry_run:
        raise RuntimeError("安全确认未完成：" + ", ".join(missing))


def metadata(args: argparse.Namespace, stats: dict[str, object]) -> dict[str, object]:
    result = {
        "schema_version": "1.0",
        "experiment": args.experiment,
        "experiment_id": args.experiment_id,
        "side": args.side,
        "direction": {"pos": "positive", "neg": "negative", "mixed": "mixed"}[args.direction],
        "repeat": args.repeat,
        "firmware_git_commit": git_commit(),
        "firmware_build": args.firmware_build,
        "protocol_version": PROTOCOL_VERSION,
        "sample_rate_target_hz": 500,
        "encoder_counts_per_wheel_rev": COUNTS_PER_REV,
        "pwm_limit": args.pwm_limit,
        "wheel_radius_m": args.wheel_radius,
        "wheel_inertia_config": args.inertia,
        "added_inertia_kg_m2": args.added_inertia,
        "bridge_driver_part": args.bridge_driver_part,
        "coast_pin_state": args.coast_pin_state,
        "brake_pin_state": args.brake_pin_state,
        "current_measurement": "unavailable",
        "motor_temperature_measurement": "unavailable",
        "fixture": args.fixture,
        "operator_notes": args.notes,
        "run_config": {
            "pwm_a": args.pwm_a,
            "pwm_b": args.pwm_b,
            "target_speed_rad_s": args.target_speed,
            "armed_ms": args.armed_ms,
            "phase_a_ms": args.phase_a_ms,
            "phase_b_ms": args.phase_b_ms,
            "coast_ms": args.coast_ms,
            "max_speed_rad_s": args.max_speed,
            "undervoltage_v": args.undervoltage,
        },
        "capture": stats,
    }
    if "battery_voltage_start_v" in stats:
        result["battery_voltage_start_v"] = stats["battery_voltage_start_v"]
        result["battery_voltage_end_v"] = stats["battery_voltage_end_v"]
    return result


def run(args: argparse.Namespace) -> int:
    if args.phase_a_ms is None:
        args.phase_a_ms = {"coast": 5000, "pwmhold": 2000, "pwmstep": 400, "statictorque": 200}[args.experiment]
    if args.phase_b_ms is None:
        args.phase_b_ms = 400 if args.experiment == "pwmstep" else 0
    require_safety_confirmation(args)
    cfg = command_for(args)
    stem = default_stem(args)
    output_dir = Path(args.output_dir)
    csv_path = output_dir / f"{stem}.csv"
    json_path = output_dir / f"{stem}.json"
    if args.dry_run:
        print(cfg)
        print(csv_path)
        return 0

    try:
        import serial
    except ImportError as error:
        raise RuntimeError("缺少 pyserial；运行 uv sync 或 pip install pyserial") from error

    port = args.serial or auto_port()
    output_dir.mkdir(parents=True, exist_ok=True)
    parser = FrameParser()
    rows: list[dict[str, object]] = []
    terminal_state = "timeout"
    start_wall = time.monotonic()
    previous_time_us: int | None = None
    armed = False

    print(f"打开 {port} @ {args.baud}；输出 {csv_path}")
    with serial.Serial(port, args.baud, timeout=0.02, write_timeout=0.5) as ser:
        ser.reset_input_buffer()
        ser.write((cfg + "\n").encode("ascii"))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not armed:
            for record in parser.feed(ser.read(ser.in_waiting or 1)):
                if record.state == 1 and record.values[2] == args.experiment_id:
                    armed = True
                    break
        if not armed:
            ser.write(b"IDSTOP\n")
            raise RuntimeError(
                "固件未进入 ARMED；请确认已烧录辨识固件，并已在 actuator_id_config.h "
                "设置 ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED=1"
            )

        ser.write(b"IDSTART\n")
        start_wall = time.monotonic()
        last_heartbeat = time.monotonic()
        capture_started = False
        try:
            while time.monotonic() - start_wall < args.timeout:
                now = time.monotonic()
                if now - last_heartbeat >= 0.4:
                    ser.write(b"IDHEART\n")
                    last_heartbeat = now
                for record in parser.feed(ser.read(ser.in_waiting or 1)):
                    if not capture_started:
                        capture_started = record.event_code == 1 and record.values[2] == args.experiment_id
                        if not capture_started:
                            continue
                    row = record.as_row(previous_time_us)
                    previous_time_us = record.values[0]
                    rows.append(row)
                    if record.state in (6, 7):
                        terminal_state = STATES[record.state]
                        raise StopIteration
        except StopIteration:
            pass
        except KeyboardInterrupt:
            terminal_state = "operator_interrupt"
            ser.write(b"IDSTOP\n")
        finally:
            if terminal_state == "timeout":
                ser.write(b"IDSTOP\n")

    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    stats = {
        "serial_port": port,
        "baud": args.baud,
        "records": len(rows),
        "terminal_state": terminal_state,
        "crc_errors": parser.crc_errors,
        "framing_errors": parser.framing_errors,
    }
    if rows:
        stats["battery_voltage_start_v"] = float(rows[0]["battery_voltage_v"])
        stats["battery_voltage_end_v"] = float(rows[-1]["battery_voltage_v"])
    json_path.write_text(json.dumps(metadata(args, stats), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"完成：{len(rows)} 样本，terminal={terminal_state}，CRC errors={parser.crc_errors}")
    return 0 if terminal_state == "COMPLETE" else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", choices=EXPERIMENT_CODES)
    parser.add_argument("--side", choices=SIDE_CODES, required=True)
    parser.add_argument("--direction", choices=("pos", "neg", "mixed"), required=True)
    parser.add_argument("--experiment-id", type=int, required=True)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--inertia", default="J0")
    parser.add_argument("--added-inertia", type=float, default=0.0)
    parser.add_argument("--pwm-a", type=int, required=True)
    parser.add_argument("--pwm-b", type=int, default=0)
    parser.add_argument("--target-speed", type=float, default=40.0, help="rad/s, coast spin-up target")
    parser.add_argument("--armed-ms", type=int, default=1000)
    parser.add_argument("--phase-a-ms", type=int)
    parser.add_argument("--phase-b-ms", type=int)
    parser.add_argument("--coast-ms", type=int, default=10_000)
    parser.add_argument("--max-speed", type=float, default=90.0)
    parser.add_argument("--undervoltage", type=float, default=11.1)
    parser.add_argument("--pwm-limit", type=int, default=1380)
    parser.add_argument("--wheel-radius", type=float, default=0.0335)
    parser.add_argument("--serial")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output-dir", default="realdata/actuator_id")
    parser.add_argument("--firmware-build", default="actuator-id-v1")
    parser.add_argument("--bridge-driver-part", default="UNCONFIRMED")
    parser.add_argument("--coast-pin-state", default="UNCONFIRMED")
    parser.add_argument("--brake-pin-state", default="UNCONFIRMED")
    parser.add_argument("--fixture", default="rigid chassis, both wheels off ground")
    parser.add_argument("--notes", default="")
    parser.add_argument("--wheels-off-ground", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


if __name__ == "__main__":
    try:
        sys.exit(run(build_parser().parse_args()))
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
