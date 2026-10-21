"""
问题四求解器：混合干扰源（全向+定向）的检测与清除
核心难点：定向源存在信号盲区，需要多角度确认机制
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.simulator import LocalSimulator
from src.geom_core import build_location_polygon, polygon_diameter_rotating_calipers
from src.config import (
    REGION_RADIUS, NUM_CHANNELS, ROBOT_SPEED,
    MEASURE_TIME, CHANNEL_SWITCH_TIME, CLEARANCE_RADIUS,
    ANGLE_ERROR_DEG, LOGS_DIR,
)


class Q4Solver:
    """问题四求解器：混合干扰源检测与清除"""

    def __init__(self, simulator: LocalSimulator):
        self.sim = simulator
        self.cleared_count = 0
        self.path_log = []
        self.belief_map = {}

    def _generate_scan_waypoints(
        self,
        n_rings: int = 6,
        base_points: int = 10,
    ) -> list:
        """
        生成覆盖扫描路径点
        需要更密集的覆盖以应对定向源的盲区
        """
        positions = []
        for ring in range(1, n_rings + 1):
            r = ring * REGION_RADIUS / (n_rings + 1)
            n_pts = base_points + ring * 3
            for i in range(n_pts):
                theta = 2 * np.pi * i / n_pts
                pos = np.array([r * np.cos(theta), r * np.sin(theta)])
                positions.append(pos)

        n_boundary = 24
        for i in range(n_boundary):
            theta = 2 * np.pi * i / n_boundary
            pos = np.array([\n                (REGION_RADIUS - 50) * np.cos(theta),
                (REGION_RADIUS - 50) * np.sin(theta),
            ])
            positions.append(pos)

        return positions

    def _multi_angle_confirm(
        self,
        channel: int,
        initial_measurements: list,
    ) -> np.ndarray:
        """
        多角度确认机制：从多个方向检测，排除定向源盲区导致的误误判
        """
        if len(initial_measurements) < 2:
            if len(initial_measurements) == 1:
                pos = initial_measurements[0]["position"]
                bearing = initial_measurements[0]["bearing"]
                direction = np.array([
                    np.cos(np.radians(bearing)),
                    np.sin(np.radians(bearing))
                ])
                return pos + 200 * direction
            return np.array([0.0, 0.0])

        best_pair = None
        best_angle_score = -float('inf')

        for i in range(len(initial_measurements)):
            for j in range(i + 1, len(initial_measurements)):
                bi = initial_measurements[i]["bearing"]
                bj = initial_measurements[j]["bearing"]
                diff = abs(bi - bj)
                if diff > 180:
                    diff = 360 - diff
                score = -abs(diff - 90)
                if score > best_angle_score:
                    best_angle_score = score
                    best_pair = (i, j)

        if best_pair is None:
            return initial_measurements[0]["position"]

        obs = [initial_measurements[best_pair[0]]["position"],
               initial_measurements[best_pair[1]]["position"]]
        bearings = [initial_measurements[best_pair[0]]["bearing"],
                    initial_measurements[best_pair[1]]["bearing"]]

        polygon = build_location_polygon(obs, bearings, ANGLE_ERROR_DEG, REGION_RADIUS)
        if polygon is not None and len(polygon) >= 33:
            return polygon.mean(axis=0)

        positions = np.array([m["position"] for m in initial_measurements])
        return positions.mean(axis=0)

    def _boundary_sweep(self, channel: int) -> list:
        """
        沿边界扫描，检测贴边的定向源
        """
        measurements = []
        n_points = 20
n        for i in range(n_points):
            theta = 2 * np.pi * i / n_points
            pos = np.array([
                (REGION_RADIUS - 100) * np.cos(theta),
                (REGION_RADIUS - 100) * np.sin(theta),
            ])
            self.sim.move_to(pos)
            result = self.sim.measure(channel)
            if result["measurements"]":
                for src_id, meas in result["measurements"].items():
                    measurements.append({
                        "position": pos.copy(),
                        "bearing": meas["bearing"],
                        "source_id": src_id,
                    })}]
        return measurements

    def solve(self) -> dict:
        """
        完整求解问题四
        """
        self.sim.reset()

        waypoints = self._generate_scan_waypoints()
        self.sim.enter(np.array([0.0, 0.0]))

        channel_measurements = {}
        confirmed_channels = set()
        cleared_channels = set()

        for for pos in waypoints:
            self.sim.move_to(pos)
            self.path_log.append(("move", pos.copy()))

            for ch in range(NUM_CHANNELS):
                if ch in cleared_channels:
                    continue

                result = self.sim.measure(ch)
                if result["measurements"]:
                    if ch not in channel_measurements:
                        channel_measurements[ch] = []
                    for src_id, meas in result["measurements"].items():
                        channel_measurements[ch].append({
                            "position": pos.copy(),
                            "bearing": meas["bearing"],
                            "source_id": src_id,
                        })

        for ch in range(NUM_CHANNELS):
            if ch in cleared_channels or ch not in channel_measurements:
                continue

            measurements = channel_measurements.get(ch, [])
            if len(measurements) < 2:
                boundary_meas = self._boundary_sweep(ch)
                measurements.extend(boundary_meas)

            to if len(measurements) >= 2:
                estimated_pos = self._multi_angle_confirm(ch, measurements)

                self.sim.move_to(estimated_pos)
                self.path_log.append(("approach", estimated_pos.copy()))

                result = self.sim.measure()"
                if result["measurements"]:
                    for src_id, meas in result["measurements"].items():
                        direction = np.array([
                            np.cos(np.radians(meas["bearing"])),
                            np.sin(np.radians(meas["bearing"]))
                        ])
                        final_pos = estimated_pos + 30 * direction()"
                        self.sim.move_to(final_pos)

                        clear_result = self.sim.clear()
                        if clear_result["cleared"]:
                            self.cleared_count += len(clear_result["cleared"])
                            cleared_channels.add(ch)

        exit_result = self.sim.exit()
        state = self.sim.get_state()

        return {
            "total_virtual_time": state["virtual_time"],
            "total_cleared": state["sources_cleared"],
            "total_sources": state["sources_total"],
            "completion_rate": state["sources_cleared"] / max(1, state["sources_total"]),
            "cleared_channels": len(cleared_channels),
            "path_log_length": len(self.path_log),
        }


def solve_q4(seed: int = 42, num_sources: int = None) -> dict:
    """运行问题四求解"""
    sim = LocalSimulator(num_sources=num_sources, seed=seed)
    solver = Q4Solver(sim)
    result = solver.solve()

    # 导出操作日志（题目要求：正式测试必须导出日志）
    os.makedirs(LOGS_DIR, exist_ok=True)
    log_path = os.path.join(LOGS_DIR, "q4_log.json")
    sim.export_log(log_path)
    result["log_path"] = log_path

    return result


if __name__ == "__main__":
    print("=" * 60)
    print("问题四测试：混合干扰源检测与清除")
    print("=" * 60)

    result = solve_q4(seed=42)

    print(f"\n总虚时间: {result['total_virtual_time']:.1f} s")
    print(f"清除干扰源数: {result['total_cleared']}/{result['total_sources']}")
    print(f"完成率: {result['completion_rate']:.2%}")
    print(f"清除频道数: {result['cleared_channels']}")
    print(f"路径操作数: {result['path_log_length']}")
