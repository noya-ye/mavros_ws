#!/usr/bin/env python3
"""Record selected ROS 2 topics until Ctrl-C."""

import argparse
import datetime as dt
import signal
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Record selected ROS 2 topics with rosbag2.")
    parser.add_argument(
        "-t", "--topic", action="append", dest="topics", required=True,
        help="topic to record; repeat for multiple topics",
    )
    parser.add_argument(
        "-o", "--output", type=Path,
        help="output directory (default: flight_record_YYYYmmdd_HHMMSS)",
    )
    parser.add_argument(
        "--storage", choices=("sqlite3", "mcap"),
        help="rosbag2 storage backend; omit to use the ROS default",
    )
    args = parser.parse_args()

    output = args.output or Path(
        "flight_record_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output.mkdir(parents=True, exist_ok=False)

    command = ["ros2", "bag", "record"]
    if args.storage:
        command += ["--storage", args.storage]
    command += ["-o", str(output / "bag"), *args.topics]

    print("Recording topics:", " ".join(args.topics), flush=True)
    print("Output:", output.resolve(), flush=True)
    print("Press Ctrl-C to stop.", flush=True)

    process = subprocess.Popen(command)
    signal.signal(signal.SIGINT, lambda *_: process.send_signal(signal.SIGINT))
    signal.signal(signal.SIGTERM, lambda *_: process.terminate())
    try:
        return process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        return process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
