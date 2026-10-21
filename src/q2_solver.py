"""
问题二求解器：第二检测点选择策略
基于几何稀释度(GDOP)理论，以交会角接近90°为准则
确定第二检测点的候选区域
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
    compute_intersection_angle,
    angle_to_direction,
)
from src.config import REGION_RADIUS, ANGLE_ERROR_DEG, FIGURES_DIR


def compute_gdop(
    observer1: np.ndarray,
    observer2: np.ndarray,
    target: np.ndarray,
) -> float:
    """
    计算几何稀释度(GDOP)
    GDOP越小，定位精度越高
    """
    dir1 = target - observer1
    dir2 = target - observer2

    dist1 = np.linalg.norm(dir1)
    dist2 = np.linalg.norm(dir2)

    if dist1 < 1e-6 or dist2 < 1e-6:
        return float('inf')

    dir1 = dir1 / dist1
    dir2 = dir2 / dist2

    sin_theta = abs(dir1[0] * dir2[1] - dir1[1] * dir2[0])
    if sin_theta < 1e-6:
        return float('inf')

    gdop = 1.0 / sin_theta * np.sqrt(1.0 / dist1**2 + 1.0 / dist2**2)
    return gdop


def find_optimal_second_observer(
    observer1: np.ndarray,
    bearing1: float,
    target_estimate: np.ndarray,
    region_radius: float = REGION_RADIUS,
    n_grid: int = 100,
) -> dict:
    """
    在极坐标网格上搜索最优第二检测点

    Parameters:
        observer1: 第一检测点坐标
        bearing1: 第一检测点测得的示向度
        target_estimate: 目标位置的粗略估计
        region_radius: 目标区域半径
        n_grid: 网格密度

    Returns:
        dict with optimal position, GDOP value, and candidate region
    """
    best_pos = None
    best_gdop = float('inf')
    gdop_map = np.full((n_grid, n_grid), np.inf)

    r_values = np.linspace(50, region_radius * 0.95, n_grid)
    theta_values = np.linspace(0, 2 * np.pi, n_grid, endpoint=False)

    candidate_positions = []

    for i, r in enumerate(r_values):
        for j, theta in enumerate(theta_values):
            pos = np.array([r * np.cos(theta), r * np.sin(theta)])

            bearing_to_target = np.degrees(np.arctan2(
                target_estimate[1] - pos[1],
                target_estimate[0] - pos[0]
            )) % 360

            bearing_diff = abs(bearing_to_target - bearing1)
            if bearing_diff > 180:
                bearing_diff = 360 - bearing_diff

            if bearing_diff > 90 + ANGLE_ERROR_DEG + 5:
                continue

            gdop = compute_gdop(pos, observer1, target_estimate)
            gdop_map[i, j] = gdop

            if gdop < best_gdop:
                best_gdop = gdop
                best_pos = pos.copy()

            candidate_positions.append((pos, gdop))

    intersection_angles = []
    for pos, gdop in candidate_positions:
        if gdop < best_gdop * 2:
            angle = compute_intersection_angle(observer1, pos, target_estimate)
            intersection_angles.append((pos, angle, gdop))

    # 兜底：若网格搜索无有效候选（全部被示向度约束过滤），
    # 用几何准则直接构造第二检测点——位于"目标→观测点1"连线方向、
    # 使交会角接近90°，即从目标出发沿垂直于视线方向偏移。
    if best_pos is None:
        dist_to_target = np.linalg.norm(target_estimate - observer1)
        bearing_to_target = np.degrees(np.arctan2(
            target_estimate[1] - observer1[1],
            target_estimate[0] - observer1[0]
        )) % 360
        # 第二检测点方位：从观测点1看，目标方位旋转90°方向、等距处
        second_bearing = (bearing_to_target + 90) % 360
        r = min(dist_to_target, region_radius * 0.95)
        best_pos = np.array([r * np.cos(np.radians(second_bearing)),
                             r * np.sin(np.radians(second_bearing))])
        best_gdop = compute_gdop(best_pos, observer1, target_estimate)

    return {
        "optimal_position": best_pos,
        "optimal_gdop": best_gdop,
        "candidate_region": candidate_positions,
        "good_candidates": intersection_angles,
        "gdop_map": gdop_map,
        "r_values": r_values,
        "theta_values": theta_values,
    }


def analyze_candidate_region_geometry(
    observer1: np.ndarray,
    bearing1: float,
    target_estimate: np.ndarray,
) -> dict:
    """
    分析候选区域的几何特征
    最优候选区域为环形扇区：
    - 与示向度方向近似垂直
    - 距离与目标距离同量级
    """
    dist_to_target = np.linalg.norm(target_estimate - observer1)
    bearing_to_target = np.degrees(np.arctan2(
        target_estimate[1] - observer1[1],
        target_estimate[0] - observer1[0]
    )) % 360

    optimal_bearing = (bearing1 + 90) % 360
    optimal_direction = angle_to_direction(optimal_bearing)

    r_min = dist_to_target * 0.3
    r_max = dist_to_target * 2.0
    r_max = min(r_max, REGION_RADIUS * 0.95)

    angular_width = 60

    return {
        "dist_to_target": dist_to_target,
        "bearing_to_target": bearing_to_target,
        "optimal_bearing": optimal_bearing,
        "optimal_direction": optimal_direction,
        "r_min": r_min,
        "r_max": r_max,
        "angular_width": angular_width,
        "center_angle": optimal_bearing,
    }


def solve_q2(
    observer1: np.ndarray,
    bearing1: float,
    target_estimate: np.ndarray = None,
    region_radius: float = REGION_RADIUS,
) -> dict:
    """
    完整求解问题二
    """
    if target_estimate is None:
        direction = angle_to_direction(bearing1)
        target_estimate = observer1 + (region_radius * 0.5) * direction

    optimal = find_optimal_second_observer(
        observer1, bearing1, target_estimate, region_radius
    )

    geometry = analyze_candidate_region_geometry(
        observer1, bearing1, target_estimate
    )

    return {
        "observer1": observer1,
        "bearing1": bearing1,
        "target_estimate": target_estimate,
        "optimal_position": optimal["optimal_position"],
        "optimal_gdop": optimal["optimal_gdop"],
        "candidate_region": optimal["candidate_region"],
        "good_candidates": optimal["good_candidates"],
        "geometry": geometry,
    }


def plot_q2_result(result: dict, save_path: str = None):
    """可视化问题二结果"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    circle = plt.Circle((0, 0), REGION_RADIUS, fill=False,
                         linestyle='-', color='black', linewidth=2)
    ax.add_patch(circle)

    obs1 = result["observer1"]
    ax.plot(obs1[0], obs1[1], 'r^', markersize=12, label='第一检测点 P1')

    target = result["target_estimate"]
    ax.plot(target[0], target[1], 'g*', markersize=15, label='目标估计位置')

    if result["optimal_position"] is not None:
        opt = result["optimal_position"]
        ax.plot(opt[0], opt[1], 'bs', markersize=12, label=f'最优第二检测点 (GDOP={result["optimal_gdop"]:.4f})')

    geo = result["geometry"]
    candidate_circle1 = plt.Circle(
        obs1, geo["r_min"], fill=False, linestyle='--', color='blue', alpha=0.3
    )
    candidate_circle2 = plt.Circle(
        obs1, geo["r_max"], fill=False, linestyle='--', color='blue', alpha=0.3
    )
    ax.add_patch(candidate_circle1)
    ax.add_patch(candidate_circle2)

    center_angle_rad = np.radians(geo["center_angle"])
    half_width_rad = np.radians(geo["angular_width"] / 2)
    theta_range = np.linspace(center_angle_rad - half_width_rad,
                              center_angle_rad + half_width_rad, 50)

    for r in [geo["r_min"], geo["r_max"]]:
        xs = r * np.cos(theta_range)
        ys = r * np.sin(theta_range)
        ax.plot(xs, ys, 'b--', alpha=0.5, linewidth=1)

    ax.annotate('', xy=obs1 + 400 * angle_to_direction(geo["bearing_to_target"]),
                xytext=obs1,
                arrowprops=dict(arrowstyle='->', color='gray', lw=1.5))

    ax.set_xlim(-REGION_RADIUS * 1.2, REGION_RADIUS * 1.2)
    ax.set_ylim(-REGION_RADIUS * 1.2, REGION_RADIUS * 1.2)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('问题二：第二检测点候选区域分析')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return fig


if __name__ == "__main__":
    print("=" * 60)
    print("问题二测试：第二检测点选择策略")
    print("=" * 60)

    obs1 = np.array([500.0, 300.0])
    bearing1 = 200.0
    target_est = np.array([-200.0, -400.0])

    result = solve_q2(obs1, bearing1, target_est)

    print(f"\n第一检测点: ({obs1[0]:.0f}, {obs1[1]:.0f})")
    print(f"示向度: {bearing1:.1f}°")
    print(f"目标估计: ({target_est[0]:.0f}, {target_est[1]:.0f})")
    print(f"\n最优第二检测点: ({result['optimal_position'][0]:.1f}, {result['optimal_position'][1]:.1f})")
    print(f"最优GDOP: {result['optimal_gdop']:.6f}")
    print(f"候选区域半径范围: [{result['geometry']['r_min']:.0f}, {result['geometry']['r_max']:.0f}] m")
    print(f"候选区域中心方位: {result['geometry']['center_angle']:.1f}°")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    save_path = os.path.join(FIGURES_DIR, "q2_result.png")
    plot_q2_result(result, save_path=save_path)
    print(f"\n可视化图已保存: {save_path}")
