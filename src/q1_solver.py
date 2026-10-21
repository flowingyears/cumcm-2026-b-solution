"""
问题一求解器：交会定位区域直径与覆盖性分析
输入：若干检测点坐标 + 对应示向度
输出：定位区域凸多边形、直径、覆盖性判断
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import src.viz  # noqa: F401  # 加载中文字体配置，让图里中文正常显示
from src.geom_core import (
    build_location_polygon,
    polygon_diameter_rotating_calipers,
    minimum_enclosing_circle,
    check_circle_coverage,
    angle_to_direction,
)
from src.config import REGION_RADIUS, ANGLE_ERROR_DEG, FIGURES_DIR


def solve_q1(
    observer_positions: list,
    bearings: list,
    error_deg: float = ANGLE_ERROR_DEG,
    region_radius: float = REGION_RADIUS,
) -> dict:
    """
    求解问题一

    Parameters:
        observer_positions: [(x1,y1), (x2,y2), ...] 检测点坐标列表
        bearings: [b1, b2, ...] 各检测点对同一干扰源的示向度（度）
        error_deg: 测角误差（±度）
        region_radius: 目标区域半径

    Returns:
        dict with keys: polygon, diameter, mec_center, mec_radius, covered, coverage_ratio
    """
    observers = [np.array(p, dtype=float) for p in observer_positions]

    polygon = build_location_polygon(
        observers=observers,
        bearings=bearings,
        error_deg=error_deg,
        region_radius=region_radius,
    )

    if polygon is None or len(polygon) < 3:
        return {
            "polygon": None,
            "diameter": 0.0,
            "mec_center": np.array([0.0, 0.0]),
            "mec_radius": 0.0,
            "covered": True,
            "coverage_ratio": 1.0,
            "num_vertices": 0,
            "area": 0.0,
            "message": "定位区域为空（示向度射线无公共交会区域）",
        }

    diameter = polygon_diameter_rotating_calipers(polygon)
    mec_center, mec_radius = minimum_enclosing_circle(polygon)
    covered, coverage_ratio = check_circle_coverage(polygon, diameter)

    return {
        "polygon": polygon,
        "diameter": diameter,
        "mec_center": mec_center,
        "mec_radius": mec_radius,
        "covered": covered,
        "coverage_ratio": coverage_ratio,
        "num_vertices": len(polygon),
        "area": _polygon_area(polygon),
    }


def _polygon_area(vertices: np.ndarray) -> float:
    """计算凸多边形面积（Shoelace公式）"""
    n = len(vertices)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i, 0] * vertices[j, 1]
        area -= vertices[j, 0] * vertices[i, 1]
    return abs(area) / 2.0


def plot_q1_result(result: dict, observers: list, bearings: list, save_path: str = None):
    """可视化问题一结果"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    circle = plt.Circle((0, 0), REGION_RADIUS, fill=False,
                         linestyle='-', color='black', linewidth=2, label='目标区域边界')
    ax.add_patch(circle)

    if result["polygon"] is not None:
        polygon = result["polygon"]
        polygon_closed = np.vstack([polygon, polygon[0]])
        ax.fill(polygon_closed[:, 0], polygon_closed[:, 1],
                alpha=0.2, color='blue', label='定位区域')
        ax.plot(polygon_closed[:, 0], polygon_closed[:, 1],
                'b-', linewidth=2)

        for i, v in enumerate(polygon):
            ax.plot(v[0], v[1], 'bo', markersize=6)
            ax.annotate(f'V{i}', (v[0], v[1]), textcoords="offset points",
                       xytext=(5, 5), fontsize=8)

    mec_circle = plt.Circle(
        result["mec_center"], result["mec_radius"],
        fill=False, linestyle='--', color='red', linewidth=1.5,
        label=f'最小包围圆 (r={result["mec_radius"]:.1f}m)'
    )
    ax.add_patch(mec_circle)

    diameter_circle = plt.Circle(
        result.get("polygon", np.array([[0, 0]])).mean(axis=0) if result["polygon"] is not None else np.array([0, 0]),
        result["diameter"] / 2,
        fill=False, linestyle=':', color='green', linewidth=1.5,
        label=f'直径圆 (D={result["diameter"]:.1f}m)'
    )
    ax.add_patch(diameter_circle)

    for i, (obs, bearing) in enumerate(zip(observers, bearings)):
        ax.plot(obs[0], obs[1], 'r^', markersize=10)
        ax.annotate(f'P{i}', (obs[0], obs[1]), textcoords="offset points",
                   xytext=(5, 5), fontsize=9, color='red')

        direction = angle_to_direction(bearing)
        arrow_end = np.array(obs) + 500 * direction
        ax.annotate('', xy=arrow_end, xytext=obs,
                    arrowprops=dict(arrowstyle='->', color='red', lw=1.5))

    ax.set_xlim(-REGION_RADIUS * 1.2, REGION_RADIUS * 1.2)
    ax.set_ylim(-REGION_RADIUS * 1.2, REGION_RADIUS * 1.2)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'问题一：定位区域 (直径D={result["diameter"]:.1f}m, '
                 f'覆盖={"是" if result["covered"] else "否"})')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return fig


if __name__ == "__main__":
    print("=" * 60)
    print("问题一测试：交会定位区域直径与覆盖性分析")
    print("=" * 60)

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
    print(f"真实干扰源位置: {true_source.tolist()}")
    print(f"三个观测点: {test_observers}")
    print(f"反推示向度: {[f'{b:.2f}' for b in test_bearings]}")

    result = solve_q1(test_observers, test_bearings)

    print(f"\n定位区域顶点数: {result['num_vertices']}")
    print(f"定位区域面积: {result['area']:.1f} m²")
    print(f"定位区域直径: {result['diameter']:.1f} m")
    print(f"最小包围圆半径: {result['mec_radius']:.1f} m")
    print(f"直径圆能否覆盖: {'是' if result['covered'] else '否'}")
    print(f"覆盖率: {result['coverage_ratio']:.4f}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    save_path = os.path.join(FIGURES_DIR, "q1_result.png")
    plot_q1_result(result, test_observers, test_bearings, save_path=save_path)
    print(f"\n可视化图已保存: {save_path}")
