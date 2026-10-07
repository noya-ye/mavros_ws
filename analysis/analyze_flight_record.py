#!/usr/bin/env python3
"""Summarize EGO command tracking and emergency holds in a ROS 2 bag DB3."""

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    db = sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True)
    topics = {i: (name, get_message(kind)) for i, name, kind in
              db.execute("SELECT id, name, type FROM topics")}
    start = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    rows = {}
    grids = []
    goal_frame = None
    for topic_id, stamp, blob in db.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
        name, cls = topics[topic_id]
        msg = deserialize_message(blob, cls)
        t = (stamp - start) / 1e9
        if name == "/position_cmd":
            p, v, a = msg.position, msg.velocity, msg.acceleration
            row = [t, p.x, p.y, v.x, v.y, a.x, a.y, msg.trajectory_id]
        elif name == "/mavros/local_position/velocity_local":
            v = msg.twist.linear
            row = [t, v.x, v.y, v.z]
        elif name == "/mavros/local_position/pose":
            p = msg.pose.position
            row = [t, p.x, p.y, p.z]
        elif name == "/fastlio2/lio_odom":
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            row = [t, p.x, p.y, v.x, v.y]
        elif name == "/mavros/setpoint_raw/local":
            p, v, a = msg.position, msg.velocity, msg.acceleration_or_force
            row = [t, p.x, p.y, v.x, v.y, a.x, a.y, msg.type_mask]
        elif name == "/mavros/state":
            row = [t, msg.armed, msg.mode]
        elif name == "/ego_2d_planner/emergency_stop":
            p = msg.position
            row = [t, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                   p.x, p.y, p.z]
        elif name == "/ego_2d_planner/selected_path":
            row = [t, len(msg.poses)]
        elif name == "/simple_2d_planner/goal":
            p = msg.pose.position
            goal_frame = msg.header.frame_id
            row = [t, p.x, p.y, p.z]
        elif name == "/ego_2d_planner/occupancy_grid":
            info = msg.info
            cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
            occupied = np.argwhere(cells >= 50)
            known_count = int(np.count_nonzero(cells >= 0))
            row = [t, info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y,
                   len(occupied), known_count, cells.size]
            grids.append((t, info, cells))
        else:
            continue
        rows.setdefault(name, []).append(row)
    db.close()
    data = {name: np.asarray(values, dtype=object if name == "/mavros/state" else float)
            for name, values in rows.items()}

    def speed_stats(values):
        return {"p50": float(np.percentile(values, 50)),
                "p90": float(np.percentile(values, 90)),
                "max": float(np.max(values))}

    report = {"duration_s": float((db_start_end(args.bag)[1] - start) / 1e9),
              "topics": {name: {"count": len(a), "start_s": float(a[0, 0]),
                                "end_s": float(a[-1, 0]),
                                "median_hz": float(1 / np.median(np.diff(a[:, 0])))
                                if len(a) > 1 else None}
                         for name, a in data.items()}}
    cmd = data["/position_cmd"]
    actual = data["/mavros/local_position/velocity_local"]
    cmd_speed = np.linalg.norm(cmd[:, 3:5].astype(float), axis=1)
    actual_speed = np.linalg.norm(actual[:, 1:3].astype(float), axis=1)
    report["speed_mps"] = {"planner": speed_stats(cmd_speed),
                           "aircraft": speed_stats(actual_speed)}
    active = (cmd[:, 0].astype(float) >= 4) & (cmd[:, 0].astype(float) <= 49)
    raw = data["/mavros/setpoint_raw/local"]
    tracking = (cmd[:, 0].astype(float) >= raw[0, 0]) & (
        cmd[:, 0].astype(float) <= raw[-1, 0])
    actual_at_cmd = np.column_stack([
        np.interp(cmd[tracking, 0].astype(float), actual[:, 0].astype(float),
                  actual[:, axis].astype(float)) for axis in (1, 2)])
    report["raw_setpoint_execution_window"] = {
        "start_s": float(raw[0, 0]), "end_s": float(raw[-1, 0]),
        "planner_speed_mps": speed_stats(cmd_speed[tracking]),
        "planner_acceleration_mps2": speed_stats(
            np.linalg.norm(cmd[tracking, 5:7].astype(float), axis=1)),
        "aircraft_speed_mps": speed_stats(np.linalg.norm(actual_at_cmd, axis=1)),
        "trajectory_ids": sorted(set(cmd[tracking, 7].astype(int).tolist())),
    }
    stop_msgs = data.get("/ego_2d_planner/emergency_stop", np.empty((0, 5)))
    unique_stops = {}
    for event in stop_msgs:
        issue_time = float(event[1]) - start / 1e9
        unique_stops[round(issue_time, 3)] = {
            "issue_s": issue_time,
            "hold_xy": [float(event[2]), float(event[3])],
        }
    report["emergency_events"] = list(unique_stops.values())
    goal = data.get("/simple_2d_planner/goal")
    lio = data.get("/fastlio2/lio_odom")
    grid_data = data.get("/ego_2d_planner/occupancy_grid")
    if grid_data is not None:
        occupied_counts = grid_data[:, 6].astype(float)
        known_counts = grid_data[:, 7].astype(float)
        report["occupancy_grid"] = {
            "count": len(grid_data),
            "occupied_cells_p50_p90_max": [float(v) for v in
                np.percentile(occupied_counts, [50, 90, 100])],
            "known_cell_fraction_p50": float(np.median(
                known_counts / grid_data[:, 8].astype(float))),
            "goal_cell_values": [],
            "aircraft_cell_values": [],
        }
        if goal is not None and len(goal):
            gx, gy = float(goal[0, 1]), float(goal[0, 2])
            report["goal"] = {"frame_id": goal_frame, "x": gx, "y": gy,
                               "z": float(goal[0, 3])}
            sampled_grids = grids[::max(1, len(grids) // 10)]
            for t, info, cells in sampled_grids:
                ix = int((gx - info.origin.position.x) / info.resolution)
                iy = int((gy - info.origin.position.y) / info.resolution)
                value = int(cells[iy, ix]) if 0 <= ix < info.width and 0 <= iy < info.height else None
                report["occupancy_grid"]["goal_cell_values"].append([float(t), value])
            if grids:
                t, info, cells = grids[0]
                ix = int((gx - info.origin.position.x) / info.resolution)
                iy = int((gy - info.origin.position.y) / info.resolution)
                x0, y0 = info.origin.position.x, info.origin.position.y
                details = {"sample_s": float(t), "frame_id": goal_frame,
                           "resolution_m": float(info.resolution),
                           "width": int(info.width), "height": int(info.height),
                           "origin_xy": [float(x0), float(y0)],
                           "goal_cell_xy": [ix, iy]}
                if 0 <= ix < info.width and 0 <= iy < info.height:
                    neighborhood = cells[max(0, iy - 5):iy + 6, max(0, ix - 5):ix + 6]
                    details["occupied_cells_within_0_5m"] = int(np.count_nonzero(neighborhood >= 50))
                    details["goal_cell"] = int(cells[iy, ix])
                report["goal_grid_context"] = details
        if lio is not None and len(lio):
            for t, info, cells in grids[::max(1, len(grids) // 10)]:
                pose = lio[np.clip(np.searchsorted(lio[:, 0].astype(float), t), 0, len(lio) - 1)]
                ix = int((float(pose[1]) - info.origin.position.x) / info.resolution)
                iy = int((float(pose[2]) - info.origin.position.y) / info.resolution)
                value = int(cells[iy, ix]) if 0 <= ix < info.width and 0 <= iy < info.height else None
                report["occupancy_grid"]["aircraft_cell_values"].append([float(t), value])
    state = data["/mavros/state"]
    report["state_changes"] = [state[i].tolist() for i in range(len(state))
                                if i == 0 or state[i, 1:] .tolist() != state[i - 1, 1:].tolist()]
    selected = data.get("/ego_2d_planner/selected_path")
    report["selected_path_message_count"] = 0 if selected is None else len(selected)
    report["path_pose_count_values"] = sorted(set(
        selected[:, 1].astype(int).tolist())) if selected is not None else []
    output = args.output or args.bag.with_name("analysis.json")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=True) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=True))


def db_start_end(bag):
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as db:
        return db.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()


if __name__ == "__main__":
    main()
