"""
Read real-time state data from WHEELTEC self-balancing robot.

Supports original firmware (A/B/C packets) and modified firmware (13-float packets).

Usage:
    python read_bt.py                          # WiFi TCP (default, 192.168.4.1:6390)
    python read_bt.py --tcp 192.168.4.1       # WiFi TCP, custom IP
    python read_bt.py -s COM3                 # Bluetooth/Serial
    python read_bt.py -s /dev/rfcomm0         # Linux Bluetooth
    python read_bt.py --raw                   # raw dump (no parsing)

Original firmware APP_Show format (alternating, flag toggles each call):
    {Aenc_L:enc_R:voltage:angle}$   —  left/right encoder speed (int), voltage×10 (int), Angle_Balance (int, °)
    {Bpitch:roll:yaw}$              —  Pitch, Roll, Yaw (int, ×10 degrees)
    {Ckp:ki:...}$                   —  PID params (only when PID_Send flag)
"""
import argparse
import csv
import time
import socket


OUTPUT_COLS = [
    "time_s", "packet_type",
    "encoder_L", "encoder_R", "voltage", "angle_balance",
    "pitch", "roll", "yaw",
]

# A packet: encoder speed × 1.1 -> display units; voltage = (raw - 1110) * 2/3
# B packet: pitch/roll/yaw are direct int values (×10 degrees)
# We store raw display values; restore actual values:
#   encoder_speed_mm_s = raw_enc / 1.1
#   voltage_v = raw_volt * 3/2 + 1110 then / 100
#   angle_deg = raw_angle
#   pitch_deg = pitch / 10.0


def parse_original(frame: str):
    """Parse A/B/C packet from original firmware. Returns dict or None."""
    if len(frame) < 2:
        return None
    ptype = frame[0]
    if ptype not in ("A", "B", "C"):
        return None

    # frame = "A123:456:789:10" or "B1:2:3"  (type char + colon-separated ints)
    data = frame[1:]  # skip type char
    if data.startswith("-"):
        data = data[1:]  # skip negative sign if first number is negative
    parts = data.split(":")
    try:
        if ptype == "A" and len(parts) >= 4:
            return {
                "packet_type": "A",
                "encoder_L": int(parts[0]),
                "encoder_R": int(parts[1]),
                "voltage": int(parts[2]),
                "angle_balance": int(parts[3]),
                "pitch": "", "roll": "", "yaw": "",
            }
        elif ptype == "B" and len(parts) >= 3:
            return {
                "packet_type": "B",
                "encoder_L": "", "encoder_R": "", "voltage": "", "angle_balance": "",
                "pitch": int(parts[0]),
                "roll": int(parts[1]),
                "yaw": int(parts[2]),
            }
        elif ptype == "C":
            return {"packet_type": "C",
                    "encoder_L":"","encoder_R":"","voltage":"","angle_balance":"",
                    "pitch":"","roll":"","yaw":""}
    except (ValueError, IndexError):
        pass
    return None


def read_tcp(host: str, port: int, duration: float, output: str, raw: bool):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    print(f"Connecting to {host}:{port}...")

    try:
        s.connect((host, port))
    except OSError:
        for p in [6390, 8080, 333, 23]:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3)
                s.connect((host, p))
                port = p
                print(f"Connected on port {p}")
                break
            except OSError:
                continue
        else:
            print("Could not connect to any port.")
            return

    print(f"Recording for {duration}s...")

    if raw:
        with open(output, "w") as f:
            t_start = time.monotonic()
            while time.monotonic() - t_start < duration:
                try:
                    data = s.recv(256)
                    if data:
                        f.write(data.decode("utf-8", errors="replace"))
                        print(data.decode("utf-8", errors="replace"), end="", flush=True)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error: {e}")
                    break
        s.close()
        print(f"\nRaw data saved to {output}")
        return

    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(OUTPUT_COLS)

        buf = ""
        t_start = time.monotonic()
        count_a = count_b = 0

        while time.monotonic() - t_start < duration:
            try:
                data = s.recv(256)
                if not data:
                    break
                buf += data.decode("utf-8", errors="replace")

                while True:
                    start = buf.find("{")
                    end = buf.find("}", start)
                    if start < 0 or end < 0:
                        break

                    frame = buf[start + 1 : end].rstrip("$")
                    buf = buf[end + 1 :]

                    parsed = parse_original(frame) if frame else None
                    if parsed is None:
                        # try floating-point format (modified firmware)
                        parts = frame.split(":")
                        try:
                            vals = [float(p) for p in parts[:13]]
                            if len(vals) >= 9:
                                t = time.monotonic() - t_start
                                writer.writerow(
                                    [f"{t:.4f}", "F"]
                                    + [f"{v:.4f}" for v in vals[:8]]
                                    + [""]
                                )
                        except ValueError:
                            continue
                    else:
                        t = time.monotonic() - t_start
                        writer.writerow([
                            f"{t:.4f}", parsed["packet_type"],
                            parsed["encoder_L"], parsed["encoder_R"],
                            parsed["voltage"], parsed["angle_balance"],
                            parsed["pitch"], parsed["roll"], parsed["yaw"],
                        ])
                        if parsed["packet_type"] == "A":
                            count_a += 1
                        elif parsed["packet_type"] == "B":
                            count_b += 1

                        total = count_a + count_b
                        if total % 100 == 0 and total > 0:
                            ang = parsed["angle_balance"]
                            print(f"  {t:.1f}s: A={count_a} B={count_b}, angle={ang}")

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    s.close()
    print(f"Done. A={count_a} B={count_b} packets -> {output}")


def read_serial(port: str, baud: int, duration: float, output: str, raw: bool):
    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial")
        return

    print(f"Opening {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=1)

    if raw:
        with open(output, "w") as f:
            t_start = time.monotonic()
            while time.monotonic() - t_start < duration:
                try:
                    n = ser.in_waiting or 1
                    data = ser.read(n)
                    if data:
                        f.write(data.decode("utf-8", errors="replace"))
                        print(data.decode("utf-8", errors="replace"), end="", flush=True)
                except Exception as e:
                    print(f"Error: {e}")
                    break
        ser.close()
        print(f"\nRaw data saved to {output}")
        return

    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(OUTPUT_COLS)

        buf = ""
        t_start = time.monotonic()
        count_a = count_b = 0

        while time.monotonic() - t_start < duration:
            try:
                n = ser.in_waiting or 1
                buf += ser.read(n).decode("utf-8", errors="replace")

                while True:
                    start = buf.find("{")
                    end = buf.find("}", start)
                    if start < 0 or end < 0:
                        break

                    frame = buf[start + 1 : end].rstrip("$")
                    buf = buf[end + 1 :]

                    parsed = parse_original(frame) if frame else None
                    if parsed is None:
                        parts = frame.split(":")
                        try:
                            vals = [float(p) for p in parts[:13]]
                            if len(vals) >= 9:
                                t = time.monotonic() - t_start
                                writer.writerow(
                                    [f"{t:.4f}", "F"]
                                    + [f"{v:.4f}" for v in vals[:8]]
                                    + [""]
                                )
                        except ValueError:
                            continue
                    else:
                        t = time.monotonic() - t_start
                        writer.writerow([
                            f"{t:.4f}", parsed["packet_type"],
                            parsed["encoder_L"], parsed["encoder_R"],
                            parsed["voltage"], parsed["angle_balance"],
                            parsed["pitch"], parsed["roll"], parsed["yaw"],
                        ])
                        if parsed["packet_type"] == "A":
                            count_a += 1
                        elif parsed["packet_type"] == "B":
                            count_b += 1

                        total = count_a + count_b
                        if total % 40 == 0 and total > 0:
                            ang = parsed["angle_balance"]
                            print(f"  {t:.1f}s: A={count_a} B={count_b}, angle={ang}")

            except Exception as e:
                print(f"Error: {e}")
                break

    ser.close()
    print(f"Done. A={count_a} B={count_b} packets -> {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WHEELTEC state data logger")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--tcp", nargs="?", const="192.168.4.1", default=None,
                       help="WiFi TCP mode (default IP: 192.168.4.1)")
    group.add_argument("-s", "--serial", type=str, default=None,
                       help="Serial port (e.g. COM3, /dev/rfcomm0)")
    parser.add_argument("-p", "--port", type=int, default=6390,
                        help="TCP port (default: 6390)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="Serial baud (default: 115200)")
    parser.add_argument("-t", "--time", type=float, default=30.0,
                        help="Duration in seconds (default: 30)")
    parser.add_argument("-o", "--output", default="wheeltc_log.csv")
    parser.add_argument("--raw", action="store_true",
                        help="Raw dump mode (no parsing, print to console)")

    args = parser.parse_args()

    if args.raw and args.serial:
        read_serial(args.serial, args.baud, args.time, args.output, raw=True)
    elif args.raw:
        read_tcp(args.tcp or "192.168.4.1", args.port, args.time, args.output, raw=True)
    elif args.serial:
        read_serial(args.serial, args.baud, args.time, args.output, raw=False)
    else:
        read_tcp(args.tcp or "192.168.4.1", args.port, args.time, args.output, raw=False)
