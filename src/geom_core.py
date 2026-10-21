"""
几何核心模块
功能：角楔构造、半平面裁剪、凸多边形交集、直径计算、最小包围圆
"""

import numpy as np
from scipy.spatial import ConvexHull
from typing import List, Tuple, Optional


def angle_to_direction(angle_deg: float) -> np.ndarray:
    """将角度(度)转换为单位方向向量"""
    rad = np.radians(angle_deg)
    return np.array([np.cos(rad), np.sin(rad)])


def build_angle_wedge(
    observer: np.ndarray,
    bearing_deg: float,
    error_deg: float = 1.0,
    half_length: float = 5000.0,
) -> np.ndarray:
    """
    构造角楔（两条射线之间的扇形区域）
    observer: 观测点坐标 [x, y]
    bearing_deg: 示向度（干扰源相对于观测点的方位角，度）
    error_deg: 测角误差（±度数）
    half_length: 射线长度（足够大即可）

    返回: 角楔区域的顶点列表（三角形: observer, left_tip, right_tip）
    """
    direction = angle_to_direction(bearing_deg)
    left_dir = angle_to_direction(bearing_deg + error_deg)
    right_dir = angle_to_direction(bearing_deg - error_deg)

    left_tip = observer + half_length * left_dir
    right_tip = observer + half_length * right_dir

    return np.array([observer, left_tip, right_tip])


def half_plane_intersection(
    half_planes: List[Tuple[np.ndarray, float]],
    bounds: Optional[List[Tuple[np.ndarray, float]]] = None,
) -> Optional[np.ndarray]:
    """
    半平面交集求解
    half_planes: [(normal, d), ...] 表示 normal·x <= d 的半平面
    bounds: 额外的边界约束

    返回: 凸多边形的顶点列表（逆时针），或None（无交集）
    """
    from scipy.optimize import linprog

    all_constraints = list(half_planes)
    if bounds:
        all_constraints.extend(bounds)

    n_constraints = len(all_constraints)
    if n_constraints == 0:
        return None

    A = np.array([c[0] for c in all_constraints])
    b = np.array([c[1] for c in all_constraints])

    A_eq = None
    b_eq = None

    c_obj = np.zeros(2)

    result = linprog(
        c_obj, A_ub=A, b_ub=b, A_eq=A_eq, b_eq=b_eq,
        bounds=(None, None), method='highs'
    )

    if not result.success:
        return None

    return result.x


def polygon_from_half_planes(
    half_planes: List[Tuple[np.ndarray, float]]
) -> Optional[np.ndarray]:
    """
    从半平面集合构造凸多边形
    使用顶点枚举法

    half_planes: [(normal, d), ...] 表示 normal·x <= d
    返回: 凸多边形顶点数组 (N, 2)，逆时针排列；None表示空集或无界
    """
    if len(half_planes) < 3:
        return None

    n = len(half_planes)
    vertices = []

    for i in range(n):
        for j in range(i + 1, n):
            n1, d1 = half_planes[i]
            n2, d2 = half_planes[j]

            det = n1[0] * n2[1] - n1[1] * n2[0]
            if abs(det) < 1e-12:
                continue

            x = (d1 * n2[1] - d2 * n1[1]) / det
            y = (d2 * n1[0] - d1 * n2[0]) / det
            point = np.array([x, y])

            feasible = True
            for k in range(n):
                if k == i or k == j:
                    continue
                nk, dk = half_planes[k]
                if nk @ point > dk + 1e-9:
                    feasible = False
                    break

            if feasible:
                vertices.append(point)

    if len(vertices) < 3:
        return None

    vertices = np.array(vertices)
    center = vertices.mean(axis=0)
    angles = np.arctan2(vertices[:, 1] - center[1], vertices[:, 0] - center[0])
    order = np.argsort(angles)
    return vertices[order]


def polygon_diameter(vertices: np.ndarray) -> float:
    """
    计算凸多边形的直径（顶点对之间的最大欧氏距离）
    使用旋转卡壳算法优化
    """
    if vertices is None or len(vertices) < 2:
        return 0.0

    n = len(vertices)
    if n == 2:
        return np.linalg.norm(vertices[0] - vertices[1])

    max_dist = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            dist = np.linalg.norm(vertices[i] - vertices[j])
            if dist > max_dist:
                max_dist = dist

    return max_dist


def _cross2d(a: np.ndarray, b: np.ndarray) -> float:
    """2维叉积: cross(a,b) = a_x*b_y - a_y*b_x"""
    return a[0] * b[1] - a[1] * b[0]


def polygon_diameter_rotating_calipers(vertices: np.ndarray) -> float:
    """
    旋转卡壳算法计算凸多边形直径 O(n)
    """
    if vertices is None or len(vertices) < 2:
        return 0.0

    n = len(vertices)
    if n == 2:
        return np.linalg.norm(vertices[0] - vertices[1])

    try:
        hull = ConvexHull(vertices)
        hull_vertices = vertices[hull.vertices]
    except Exception:
        hull_vertices = vertices

    n = len(hull_vertices)
    if n < 2:
        return 0.0

    max_dist = 0.0

    j = 1
    for i in range(n):
        next_i = (i + 1) % n
        edge = hull_vertices[next_i] - hull_vertices[i]

        while True:
            next_j = (j + 1) % n
            cross_curr = _cross2d(edge, hull_vertices[j] - hull_vertices[i])
            cross_next = _cross2d(edge, hull_vertices[next_j] - hull_vertices[i])
            if cross_next > cross_curr:
                j = next_j
            else:
                break

        dist_ij = np.linalg.norm(hull_vertices[i] - hull_vertices[j])
        dist_next = np.linalg.norm(hull_vertices[next_i] - hull_vertices[j])
        max_dist = max(max_dist, dist_ij, dist_next)

    return max_dist


def minimum_enclosing_circle(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Welzl算法求最小包围圆
    返回: (圆心, 半径)
    """
    if points is None or len(points) == 0:
        return np.array([0.0, 0.0]), 0.0

    pts = list(points)
    np.random.shuffle(pts)

    def _welzl(P, R, n):
        if n == 0 or len(R) == 3:
            return _trivial_circle(R)

        p = P[n - 1]
        center, radius = _welzl(P, R, n - 1)

        if center is not None and np.linalg.norm(p - center) <= radius + 1e-10:
            return center, radius

        return _welzl(P, R + [p], n - 1)

    def _trivial_circle(R):
        if len(R) == 0:
            return None, 0.0
        elif len(R) == 1:
            return R[0], 0.0
        elif len(R) == 2:
            center = (R[0] + R[1]) / 2
            return center, np.linalg.norm(R[0] - R[1]) / 2
        else:
            return _circle_from_three(R[0], R[1], R[2])

    def _circle_from_three(a, b, c):
        ax, ay = a
        bx, by = b
        cx, cy = c

        D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(D) < 1e-12:
            pts = np.array([a, b, c])
            max_dist = 0
            best_pair = (0, 1)
            for i in range(3):
                for j in range(i + 1, 3):
                    d = np.linalg.norm(pts[i] - pts[j])
                    if d > max_dist:
                        max_dist = d
                        best_pair = (i, j)
            center = (pts[best_pair[0]] + pts[best_pair[1]]) / 2
            return center, max_dist / 2

        ux = ((ax * ax + ay * ay) * (by - cy) +
              (bx * bx + by * by) * (cy - ay) +
              (cx * cx + cy * cy) * (ay - by)) / D
        uy = ((ax * ax + ay * ay) * (cx - bx) +
              (bx * bx + by * by) * (ax - cx) +
              (cx * cx + cy * cy) * (bx - ax)) / D

        center = np.array([ux, uy])
        radius = np.linalg.norm(a - center)
        return center, radius

    return _welzl(pts, [], len(pts))


def build_location_polygon(
    observers: List[np.ndarray],
    bearings: List[float],
    error_deg: float = 1.0,
    region_radius: float = 1800.0,
) -> Optional[np.ndarray]:
    """
    根据多个观测点的示向度构造定位区域（凸多边形）

    observers: 观测点坐标列表
    bearings: 各观测点对同一干扰源的示向度列表（度）
    error_deg: 测角误差
    region_radius: 目标区域半径（用于边界约束）

    返回: 凸多边形顶点数组
    """
    half_planes = []

    for obs, bearing in zip(observers, bearings):
        direction = angle_to_direction(bearing)
        normal = np.array([-direction[1], direction[0]])

        left_dir = angle_to_direction(bearing + error_deg)
        right_dir = angle_to_direction(bearing - error_deg)

        left_normal = np.array([-left_dir[1], left_dir[0]])
        right_normal = np.array([-right_dir[1], right_dir[0]])

        d_left = left_normal @ obs
        d_right = right_normal @ obs

        half_planes.append((left_normal, d_left))
        half_planes.append((-right_normal, -d_right))

    boundary_planes = [
        (np.array([1, 0]), region_radius),
        (np.array([-1, 0]), region_radius),
        (np.array([0, 1]), region_radius),
        (np.array([0, -1]), region_radius),
    ]
    half_planes.extend(boundary_planes)

    return polygon_from_half_planes(half_planes)


def check_circle_coverage(
    polygon_vertices: np.ndarray,
    diameter: float,
) -> Tuple[bool, float]:
    """
    判断以定位区域直径为直径的圆是否能完全覆盖该区域

    返回: (是否覆盖, 覆盖率)
    """
    if polygon_vertices is None or len(polygon_vertices) < 3:
        return True, 1.0

    center, mec_radius = minimum_enclosing_circle(polygon_vertices)

    circle_radius = diameter / 2

    covered = mec_radius <= circle_radius + 1e-9

    coverage_ratio = min(1.0, (circle_radius / mec_radius) ** 2) if mec_radius > 0 else 1.0

    return covered, coverage_ratio


def compute_intersection_angle(
    observer1: np.ndarray,
    observer2: np.ndarray,
    target: np.ndarray,
) -> float:
    """
    计算两条视线在目标处的交会角（度）
    """
    dir1 = target - observer1
    dir2 = target - observer2

    dir1 = dir1 / (np.linalg.norm(dir1) + 1e-12)
    dir2 = dir2 / (np.linalg.norm(dir2) + 1e-12)

    cos_angle = np.clip(np.dot(dir1, dir2), -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))
