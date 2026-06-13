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
        if column not in rows[0]:
            continue
        values = [row[column] for row in rows]
        print(
            f"{column:9s} min {min(values): .5f} "
            f"max {max(values): .5f} last {values[-1]: .5f}"
        )


def print_reason_summary(rows):
    reason_columns = {
        "fs_rsn": {
            0: "none",
            1: "rc invalid",
            2: "rc failsafe",
        },
        "arm_deny": {
            0: "none",
            1: "rc invalid",
            2: "rc failsafe",
            3: "arm switch low",
            4: "throttle high",
            5: "battery invalid",
            6: "battery unsafe",
        },
        "stop_rsn": {
            0: "none",
            1: "battery critical fault",
            2: "battery immediate stop",
            3: "failsafe",
            4: "disarmed",
            5: "imu not ready",
            6: "state invalid",
        },
    }

    print("\nreason summary:")
    for column, labels in reason_columns.items():
        if column not in rows[0]:
            continue

        counts = {}
        for row in rows:
            code = int(row[column])
            counts[code] = counts.get(code, 0) + 1

        text = ", ".join(
            f"{code}:{labels.get(code, 'unknown')}={count}"
            for code, count in sorted(counts.items())
        )
        print(f"{column}: {text}")


def print_tuning_summary(rows):
    print("\ntuning summary:")

    for axis, target, measured, error, saturated in (
        ("roll", "t_roll", "cor_roll", "re", "rs"),
        ("pitch", "t_pitch", "cor_pitch", "pe", "ps"),
        ("yaw", "t_yaw", "cor_yaw", "ye", "ys"),
    ):
        if target not in rows[0] or measured not in rows[0]:
            continue

        active = [
            row for row in rows
            if abs(row[target]) > 1.0 or abs(row[measured]) > 1.0
        ]
        if not active:
            print(f"{axis}: no active samples")
            continue

        if error in rows[0]:
            abs_error = [abs(row[error]) for row in active]
            mean_abs_error = sum(abs_error) / len(abs_error)
            max_abs_error = max(abs_error)
            sat_count = sum(1 for row in active if saturated in row and row[saturated] > 0.5)
            print(
                f"{axis}: mean abs error {mean_abs_error:.2f} deg/s, "
                f"max {max_abs_error:.2f} deg/s, saturated {sat_count}/{len(active)}"
            )

    if "m_span" in rows[0]:
        spans = [row["m_span"] for row in rows]
        print(f"motor span: max {max(spans):.3f}, last {spans[-1]:.3f}")

    if "thr_lim" in rows[0]:
        limits = [row["thr_lim"] for row in rows]
        print(f"throttle limit: min {min(limits):.3f}, last {limits[-1]:.3f}")


def print_active_rows(rows):
    print("\nactive rows:")
    print("time thr gR gP gY estR estP cR cP cY m1 m2 m3 m4 stop z")
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
            f"{row.get('stop_rsn', 0.0):.0f} {row.get('truth_z', 0.0):+.2f}"
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
            "fs",
            "fs_rsn",
            "arm_deny",
            "stop_rsn",
            "imu_dt",
            "ctrl_dt",
            "rc_age",
            "rc_thr",
            "g_roll",
            "g_pitch",
            "g_yaw",
            "est_roll",
            "est_pitch",
            "c_roll",
            "c_pitch",
            "c_yaw",
            "re",
            "pe",
            "ye",
            "m_span",
            "thr_lim",
            "bat_v",
            "bat_a",
            "bat_warn",
            "bat_flt",
            "m1",
            "m2",
            "m3",
            "m4",
            "truth_z",
        ),
    )
    print_reason_summary(rows)
    print_tuning_summary(rows)
    print_sign_check(rows)

    if args.active:
        print_active_rows(rows)


if __name__ == "__main__":
    main()
