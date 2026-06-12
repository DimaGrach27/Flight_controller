#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


def load_rows(path: Path):
    with path.open(newline="") as f:
        rows = []
        for row in csv.DictReader(f):
            if not row.get("time_ms"):
                continue
            rows.append({key: float(value) for key, value in row.items()})
        return rows


def print_ranges(rows, columns):
    for column in columns:
        values = [row[column] for row in rows]
        print(
            f"{column:9s} min {min(values): .5f} "
            f"max {max(values): .5f} last {values[-1]: .5f}"
        )


def print_active_rows(rows):
    print("\nactive rows:")
    print("time thr gR gP gY estR estP cR cP cY m1 m2 m3 m4 z")
    for row in rows:
        active = (
            row["rc_thr"] > 0.0
            or abs(row["c_roll"]) > 1e-6
            or abs(row["c_pitch"]) > 1e-6
            or abs(row["c_yaw"]) > 1e-6
        )
        if not active:
            continue

        print(
            f"{row['time_ms']:6.0f} {row['rc_thr']:.3f} "
            f"{row['g_roll']:+7.3f} {row['g_pitch']:+7.3f} {row['g_yaw']:+7.3f} "
            f"{row['est_roll']:+7.3f} {row['est_pitch']:+7.3f} "
            f"{row['c_roll']:+6.3f} {row['c_pitch']:+6.3f} {row['c_yaw']:+6.3f} "
            f"{row['m1']:.3f} {row['m2']:.3f} {row['m3']:.3f} {row['m4']:.3f} "
            f"{row['truth_z']:+.2f}"
        )


def print_sign_check(rows):
    print("\nsign check:")
    for control, gyro in (
        ("c_roll", "g_roll"),
        ("c_pitch", "g_pitch"),
        ("c_yaw", "g_yaw"),
    ):
        pairs = [
            (row[control], row[gyro])
            for row in rows
            if row["rc_thr"] > 0.1 and abs(row[control]) > 1e-6
        ]
        if not pairs:
            print(f"{control}/{gyro}: no active samples")
            continue

        same = sum(1 for command, rate in pairs if command * rate > 0.0)
        opposite = sum(1 for command, rate in pairs if command * rate < 0.0)
        print(
            f"{control}/{gyro}: opposite={opposite} same={same} n={len(pairs)} "
            "(rate damping normally wants opposite signs)"
        )

    print("\ncommand -> next gyro delta:")
    for current, next_row in zip(rows, rows[1:]):
        if current["rc_thr"] <= 0.1:
            continue
        if abs(current["c_roll"]) <= 0.005 and abs(current["c_pitch"]) <= 0.005:
            continue

        print(
            f"{current['time_ms']:6.0f} "
            f"cR={current['c_roll']:+.3f} "
            f"gR={current['g_roll']:+.3f}->{next_row['g_roll']:+.3f} "
            f"d={next_row['g_roll'] - current['g_roll']:+.3f} | "
            f"cP={current['c_pitch']:+.3f} "
            f"gP={current['g_pitch']:+.3f}->{next_row['g_pitch']:+.3f} "
            f"d={next_row['g_pitch'] - current['g_pitch']:+.3f}"
        )


def main():
    parser = argparse.ArgumentParser(description="Analyze imu_hil_log.csv")
    parser.add_argument(
        "path",
        nargs="?",
        default="/Users/dhrachov/Projects/Embedded/Flight_controller_v1/Simulator/GazeboHil/imu_hil_log.csv",
        type=Path,
    )
    parser.add_argument("--active", action="store_true", help="print active rows")
    args = parser.parse_args()

    rows = load_rows(args.path)
    if not rows:
        raise SystemExit(f"{args.path} has no data rows")

    print(f"rows {len(rows)} time {rows[0]['time_ms']:.0f}..{rows[-1]['time_ms']:.0f} ms")
    print_ranges(
        rows,
        (
            "f_mode",
            "armed",
            "imu_dt",
            "rc_thr",
            "g_roll",
            "g_pitch",
            "g_yaw",
            "est_roll",
            "est_pitch",
            "c_roll",
            "c_pitch",
            "c_yaw",
            "m1",
            "m2",
            "m3",
            "m4",
            "truth_z",
        ),
    )
    print_sign_check(rows)

    if args.active:
        print_active_rows(rows)


if __name__ == "__main__":
    main()
