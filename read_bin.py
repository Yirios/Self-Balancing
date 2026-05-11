"""
Receive 200 Hz binary telemetry from WHEELTEC robot over WiFi TCP.

Protocol: [0xDD] [10×float32 LE] [XOR checksum] = 42 bytes
Floats: theta_L,theta_R,theta_1,theta_2,thetadot_L,thetadot_R,thetadot_1,thetadot_2,u_L,u_R

Usage:
    python read_bin.py                  # WiFi TCP (default: 192.168.4.1:6390)
    python read_bin.py -t 30 -o log.csv # 30 seconds, save to log.csv
"""
import argparse
import socket
import struct
import csv
import time

SYNC = 0xDD
PACKET_SIZE = 42
FIELDS = [
    "time_s",
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
    "u_L", "u_R",
]


def receive(host: str, port: int, duration: float, output: str):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    print(f"Connecting to {host}:{port}...")

    try:
        s.connect((host, port))
    except OSError:
        for p in [6390, 8080, 333]:
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
            print("Could not connect.")
            return

    print(f"Receiving binary packets for {duration}s...")

    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)

        buf = b""
        t_start = time.monotonic()
        count = 0
        lost = 0
        last_seq = -1

        while time.monotonic() - t_start < duration:
            try:
                data = s.recv(512)
                if not data:
                    break
                buf += data

                while len(buf) >= PACKET_SIZE:
                    sync_idx = buf.find(bytes([SYNC]))
                    if sync_idx < 0:
                        buf = buf[-PACKET_SIZE:]  # keep tail
                        break

                    if len(buf) - sync_idx < PACKET_SIZE:
                        break  # incomplete

                    raw = buf[sync_idx : sync_idx + PACKET_SIZE]

                    # Verify checksum
                    ck = 0
                    for b in raw[:-1]:
                        ck ^= b
                    if ck != raw[-1]:
                        lost += 1
                        buf = buf[sync_idx + 1 :]
                        continue

                    floats = struct.unpack("<10f", raw[1:41])
                    t = time.monotonic() - t_start
                    writer.writerow([f"{t:.4f}"] + [f"{v:.6f}" for v in floats])
                    count += 1

                    if count % 500 == 0:
                        print(
                            f"  {t:.1f}s: {count} pkts ({count/t:.0f} Hz), "
                            f"lost={lost}, "
                            f"theta_1={floats[2]:.4f} rad"
                        )

                    buf = buf[sync_idx + PACKET_SIZE :]

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    s.close()
    elapsed = time.monotonic() - t_start
    print(
        f"Done. {count} packets ({count/elapsed:.0f} Hz), "
        f"{lost} checksum errors -> {output}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive 200 Hz binary telemetry from WHEELTEC robot"
    )
    parser.add_argument("--host", default="192.168.4.1", help="WiFi module IP")
    parser.add_argument("-p", "--port", type=int, default=6390, help="TCP port")
    parser.add_argument("-t", "--time", type=float, default=30.0, help="Duration (s)")
    parser.add_argument("-o", "--output", default="binary_log.csv", help="Output CSV")
    args = parser.parse_args()
    receive(args.host, args.port, args.time, args.output)
