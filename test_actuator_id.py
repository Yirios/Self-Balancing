import csv
import struct
import tempfile
import unittest
from pathlib import Path

import actuator_id_capture as capture
from check_actuator_id import check_file


def make_frame(values: tuple[int, ...]) -> bytes:
    payload = capture.PAYLOAD.pack(*values)
    header = capture.SYNC + bytes([
        capture.PROTOCOL_VERSION, capture.FRAME_RECORD, len(payload)
    ])
    body = header + payload
    return body + struct.pack("<H", capture.crc16_ccitt(body))


class ProtocolTests(unittest.TestCase):
    def test_fragmented_frame_round_trip(self):
        values = (1_000_000, 0, 101, 1, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 12_100, 0, 1_000_000)
        frame = make_frame(values)
        parser = capture.FrameParser()
        records = parser.feed(b"junk" + frame[:17])
        records += parser.feed(frame[17:])
        self.assertEqual([record.values for record in records], [values])
        self.assertEqual(parser.crc_errors, 0)

    def test_crc_failure_resynchronizes(self):
        values = (1_000_000, 0, 101, 1, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 12_100, 0, 1_000_000)
        bad = bytearray(make_frame(values))
        bad[20] ^= 1
        parser = capture.FrameParser()
        records = parser.feed(bytes(bad) + make_frame(values))
        self.assertEqual(len(records), 1)
        self.assertEqual(parser.crc_errors, 1)


class QualityCheckTests(unittest.TestCase):
    def test_valid_minimal_run(self):
        rows = []
        states = [("ARMED", 1), ("DRIVE_HOLD", 2), ("COAST", 3), ("COMPLETE", 5)]
        for sample_id, (state, event) in enumerate(states):
            row = {field: "0" for field in capture.CSV_FIELDS}
            row.update({
                "time_us": str(1_000_000 + sample_id * 2_000),
                "sample_id": str(sample_id),
                "experiment_id": "101",
                "state": state,
                "previous_state": states[max(0, sample_id - 1)][0],
                "event_code": str(event),
                "bridge_mode_L": "COAST",
                "bridge_mode_R": "COAST",
                "wheel_speed_L_rad_s": "0",
                "wheel_speed_R_rad_s": "0",
                "battery_voltage_v": "12.1",
            })
            rows.append(row)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "coast_L_pos_J0_r01.csv"
            with path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=capture.CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            self.assertTrue(check_file(path)["valid"])


if __name__ == "__main__":
    unittest.main()
