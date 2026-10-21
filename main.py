"""
2026年高教社杯全国大学生数学建模竞赛 B题
无线电干扰源的快速自动定位与清除
主入口
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.config import *
from src.geom_core import *
from src.simulator import LocalSimulator
from src.q1_solver import solve_q1, plot_q1_result
from src.q2_solver import solve_q2, plot_q2_result
from src.q3_solver import solve_q3
from src.q4_solver import solve_q4


def run_all():
    print("=" * 70)
    print("  2026年高教社杯全国大学生数学建模竞赛 B题")
    print("  无线电干扰源的快速自动定位与清除")
    print("=" * 70)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(FIGURES_DIR, exist_ok=True)
    os.makedirs(LOGS_DIR, exist_ok=True)

    print("\n" + "=" * 70)
    print("  问题一：交会定位区域直径与覆盖性分析")
    print("=" * 70)

    # 真实干扰源位置，由三个观测点反推示向度（保证射线交会有效）
    true_source = np.array([-200.0, 200.0])
    test_observers = [
        (500, 300),
        (-400, 600),
        (200, -500),
    ]
    test_bearings = [
        float(np.degrees(np.arctan2(
            true_source[1] - o[1], true_source[0] - o[0])) % 360)
        for o in test_observers
    ]

    q1_result = solve_q1(test_observers, test_bearings)
    print(f"  定位区域顶点数: {q1_result['num_vertices']}")
    print(f"  定位区域面积:   {q1_result['area']:.1f} m²")
    print(f"  定位区域直径:   {q1_result['diameter']:.1f} m")
    print(f"  最小包围圆半径: {q1_result['mec_radius']:.1f} m")
    print(f"  直径圆能否覆盖: {'是' if q1_result['covered'] else '否'}")
    print(f"  覆盖率:         {q1_result['coverage_ratio']:.4f}")

    plot_q1_result(
        q1_result, test_observers, test_bearings,
        save_path=os.path.join(FIGURES_DIR, "q1_result.png")
    )
    print("  [OK] 问题一可视化已保存")

    print("\n" + "=" * 70)
    print("  问题二：第二检测点选择策略")
    print("=" * 70)

    obs1 = np.array([500.0, 300.0])
    bearing1 = 200.0
    target_est = np.array([-200.0, -400.0])

    q2_result = solve_q2(obs1, bearing1, target_est)
    print(f"  第一检测点:     ({obs1[0]:.0f}, {obs1[1]:.0f})")
    print(f"  示向度:         {bearing1:.1f}°")
    print(f"  目标估计:       ({target_est[0]:.0f}, {target_est[1]:.0f})")
    print(f"  最优第二检测点: ({q2_result['optimal_position'][0]:.1f}, "
          f"{q2_result['optimal_position'][1]:.1f})")
    print(f"  最优GDOP:       {q2_result['optimal_gdop']:.6f}")

    plot_q2_result(
        q2_result,
        save_path=os.path.join(FIGURES_DIR, "q2_result.png")
    )
    print("  [OK] 问题二可视化已保存")

    print("\n" + "=" * 70)
    print("  问题三：全向干扰源快速搜索与清除")
    print("=" * 70)

    q3_result = solve_q3(seed=42)
    print(f"  总虚拟时间:   {q3_result['total_virtual_time']:.1f} s")
    print(f"  清除干扰源:   {q3_result['total_cleared']}/{q3_result['total_sources']}")
    print(f"  完成率:       {q3_result['completion_rate']:.2%}")
    print(f"  发现频道数:   {q3_result['discovered_channels']}")

    print("\n" + "=" * 70)
    print("  问题四：混合干扰源检测与清除")
    print("=" * 70)

    q4_result = solve_q4(seed=42)
    print(f"  总虚拟时间:   {q4_result['total_virtual_time']:.1f} s")
    print(f"  清除干扰源:   {q4_result['total_cleared']}/{q4_result['total_sources']}")
    print(f"  完成率:       {q4_result['completion_rate']:.2%}")
    print(f"  清除频道数:   {q4_result['cleared_channels']}")

    print("\n" + "=" * 70)
    print("  全部问题求解完成！")
    print("=" * 70)

    return {
        "q1": q1_result,
        "q2": q2_result,
        "q3": q3_result,
        "q4": q4_result,
    }


if __name__ == "__main__":
    run_all()