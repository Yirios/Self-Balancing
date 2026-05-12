"""
Receive binary telemetry from WHEELTEC robot over USB serial (USART1).

Protocol: [0xDD] [16×float32 LE] [XOR checksum] = 66 bytes
Floats: theta_L,theta_R,theta_1,theta_2,thetadot_L/R/dot_1/dot_2,u_L,u_R,
        Target_theta_L/R, TargetVal_L/R, PWM_L/R

Usage:
    python read_bin.py                       # auto-detect USB serial port
    python read_bin.py -s COM3               # Windows
    python read_bin.py -s /dev/ttyUSB0       # Linux
"""
import argparse
import struct
import csv
import time
import sys

SYNC = 0xDD
PACKET_SIZE = 66
FIELDS = [
    "time_s",
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
    "u_L", "u_R",
    "Target_theta_L", "Target_theta_R",
    "TargetVal_L", "TargetVal_R",
    "PWM_L", "PWM_R",
]


def read_serial(port: str, baud: int, duration: float, output: str):
    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial")
        return

    print(f"Opening {port} @ {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)

    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)

        buf = b""
        t_start = time.perf_counter()
        count = lost = 0

        while time.perf_counter() - t_start < duration:
            n = ser.in_waiting or 1
            buf += ser.read(n)

            while len(buf) >= PACKET_SIZE:
                sync_idx = buf.find(bytes([SYNC]))
                if sync_idx < 0:
                    buf = buf[-PACKET_SIZE:]
                    break
                if len(buf) - sync_idx < PACKET_SIZE:
                    break

                raw = buf[sync_idx : sync_idx + PACKET_SIZE]
                ck = 0
                for b in raw[:-1]:
                    ck ^= b
                if ck != raw[-1]:
                    lost += 1
                    buf = buf[sync_idx + 1 :]
                    continue

                floats = struct.unpack("<16f", raw[1:65])
                t = time.perf_counter() - t_start
                writer.writerow([f"{t:.6f}"] + [f"{v:.6f}" for v in floats])
                count += 1

                if count % 500 == 0:
                    print(f"  {t:.1f}s: {count} pkts ({count/t:.0f} Hz), lost={lost}")

                buf = buf[sync_idx + PACKET_SIZE :]

    ser.close()
    elapsed = time.perf_counter() - t_start
    print(f"Done. {count} pkts ({count/elapsed:.0f} Hz), {lost} chk errors -> {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive 200 Hz binary telemetry from WHEELTEC robot"
    )
    parser.add_argument("-s", "--serial", type=str, default=None,
                        help="USB serial port (e.g. COM3, /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=460800)
    parser.add_argument("-t", "--time", type=float, default=30.0, help="Duration (s)")
    parser.add_argument("-o", "--output", default="binary_log.csv", help="Output CSV")
    parser.add_argument("--tcp", type=str, default=None, help="WiFi IP (legacy)")
    parser.add_argument("-p", "--port", type=int, default=6390)
    args = parser.parse_args()

    if args.tcp:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((args.tcp, args.port))
        print(f"WiFi mode: {args.tcp}:{args.port}")
        buf = b""
        t0 = time.perf_counter()
        count = 0
        with open(args.output, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(FIELDS)
            while time.perf_counter() - t0 < args.time:
                buf += s.recv(512)
                while len(buf) >= PACKET_SIZE:
                    idx = buf.find(bytes([SYNC]))
                    if idx < 0: buf = buf[-PACKET_SIZE:]; break
                    if len(buf) - idx < PACKET_SIZE: break
                    raw = buf[idx:idx+PACKET_SIZE]
                    buf = buf[idx+PACKET_SIZE:]
                    ck = 0
                    for b in raw[:-1]: ck ^= b
                    if ck != raw[-1]: continue
                    vals = struct.unpack("<16f", raw[1:65])
                    w.writerow([f"{time.perf_counter()-t0:.6f}"] + [f"{v:.6f}" for v in vals])
                    count += 1
        s.close()
        print(f"Done. {count} pkts -> {args.output}")
    else:
        port = args.serial
        if port is None:
            # auto-detect
            import glob
            candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
            if not candidates:
                candidates = [f"COM{i}" for i in range(1, 20)]
                print("Auto-detect not supported on this platform. Use -s COMx")
                sys.exit(1)
            port = candidates[0]
            print(f"Auto-detected: {port}")
        read_serial(port, args.baud, args.time, args.output)
