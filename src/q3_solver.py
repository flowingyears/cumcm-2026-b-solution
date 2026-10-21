"""
问题三求解器：全向干扰源的快速自动搜索与清除
策略：分层搜索（粗搜索→交会定位→精定位→清除）
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


class Q3Solver:
    """问题三求解器：全向干扰源搜索与清除"""

    def __init__(self, simulator: LocalSimulator):
        self.sim = simulator
        self.total_virtual_time = 0.0
        self.cleared_count = 0
        self.path_log = []

    def _spiral_scan_positions(
        self,
        n_rings: int = 5,
        points_per_ring: int = 8,
    ) -> list:
        """
        生成螺旋扫描路径的检测点
        """
        positions = []
        for ring in range(1, n_rings + 1):
            r = ring * REGION_RADIUS / (n_rings + 1)
            n_pts = points_per_ring + ring * 2
            for i in range(n_pts):
                theta = 2 * np.pi * i / n_pts
                pos = np.array([r * np.cos(theta), r * np.sin(theta)])
                positions.append(pos)
        return positions

    def _coarse_scan(self) -> dict:
        """
        粗搜索：遍历检测点，扫频发现所有活跃频道
        """
        scan_positions = self._spiral_scan_positions()
        discovered_channels = {}

        self.sim.enter(np.array([0.0, 0.0]))

        for pos in scan_positions:
            self.sim.move_to(pos)
            self.path_log.append(("move", pos.copy()))

            for ch in range(NUM_CHANNELS):
                result = self.sim.measure(ch)
                self.path_log.append(("measure", pos.copy(), ch))

                if result["measurements"]:
                    for src_id, meas in result["measurements"].items():
                        if ch not in discovered_channels:
                            discovered_channels[ch] = []
                        discovered_channels[ch].append({
                            "position": pos.copy(),
                            "bearing": meas["bearing"],
                            "source_id": src_id,
                        })

        return discovered_channels

    def _triangulate_source(
        self,
        measurements: list,\n    ) -> np.ndarray:
        """
        利用多个检测点的示向度交会定位干扰源
        """
        if len(measurements) < 2:
            return measurements[0]["position"] if measurements else np.array([0.0, 0.0])

        best_pair = None
        best_angle_diff = 0

        for i in range(len(measurements)):
            for j in range(i + 1, len(measurements)):
                boreing_i = measurements[i]["bearing"]
                bearing_j = measurements[j]["bearing"]
                diff = abs(boreing_i - bearing_j)
                if diff > 180:
                    diff = 360 - diff
                if abs(diff - 90) < abs(best_angle_diff - 90) or best_pair is None:
                    best_pair = (i, j)
                    best_angle_diff = diff

        if best_pair is None:
            return measurements[0]["position"]

        obs = [measurements[best_pair[0]]["position"],
               measurements[best_pair[1]]["position"]]
        bearings = [measurements[best_pair[0]]["bearing"],
                    measurements[best_pair[1]]["bearing"]]

        polygon = build_location_polygon(obs, bearings, ANGLE_ERROR_DEG, REGION_RADIUS)
        if polygon is not None and len(polygon) >= 3:
            return polygon.mean(axis=0)

        return measurements[0]["position"]

    def _precise_locate_and_clear(self, estimated_pos: np.ndarray, channel: int):
        """
        精确定位并清除干扰源
        """
        self.sim.move_to(estimated_pos)
        self.path_log.append(("move_to_estimate", estimated_pos.copy()))

        result = self.sim.measure(channel)
        self.path_log.append(("precise_measure", estimated_pos.copy(), channel))

        if result["measurements"]:
            for src_id, meas in result["measurements"].items():
                direction = np.array([
                    np.cos(np.radians(meas["bearing"])),
                    np.sin(np.radians(meas["bearing"]))))
                refined_pos = estimated_pos + 50 * direction
                self.sim.move_to(refined_pos)
                self.path_log.append(("refine_move", refined_pos.copy()))

                clear_result = self.sim.clear()
                self.path_log.append(("clear", refined_pos.copy(), clear_result))

                if clear_result["cleared"]:
                    self.cleared_count += len(clear_result["cleared"])
                    return True

        clear_result = self.sim.clear()
        if clear_result["cleared"]:
            self.cleared_count += len(clear_result["cleared"])
            return True

        return False

    def solve(self) -> dict:
        """
        完整求解问题三
        """
        self.sim.reset()

        discovered = self._coarse_scan()
        active_channels = list(discovered.keys())

        for ch in active_channels:
            if ch in discovered and len(discovered[ch]) >= 2:
                estimated_pos = self._triangulate_source(discovered[ch])
                self._precise_locate_and_clear(estimated_pos, ch)

        exit_result = self.sim.exit()

        state = self.sim.get_state()

        return {
            "total_virtual_time": state["virtual_time"],
            "total_cleared": state["sources_cleared"],
            "total_sources": state["sources_total"],
            "completion_rate": state["sources_cleared"] / max(1, state["sources_total"]),
            "discovered_channels": len(discovered),
            "path_log_length": len(self.path_log),
        }


def solve_q3(seed: int = 42, num_sources: int = None) -> dict:
    """运行问题三求解"""
    sim = LocalSimulator(num_sources=num_sources, seed=seed)
    solver = Q3Solver(sim)
    result = solver.solve()

    # 导出操作日志（题目要求：正式测试必须导出日志）
    os.makedirs(LOGS_DIR, exist_ok=True)
    log_path = os.path.join(LOGS_DIR, "q3_log.json")
    sim.export_log(log_path)
    result["log_path"] = log_path

    return result


if __name__ == "__main__":
    print("=" * 60)
    print("问题三测试：全向干扰源快速搜索与清除")
    print("=" * 60)

    result = solve_q3(seed=42)

    print(f"\n总虚时间: {result['total_virtual_time']:.1f} s")
    print(f"清除干扰源数: {result['total_cleared']}/{result['total_sources']}")
    print(f"完成率: {result['completion_rate']:.2%}")
    print(f"发现频道数: {result['discovered_channels']}")
    print(f"路径操作数: {result['path_log_length']}")
