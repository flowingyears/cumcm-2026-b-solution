# 解题思路详解

## 总体策略

本题的核心挑战是：在未知干扰源位置和数量的情况下，用机器狗通过示向度测量快速定位并清除所有干扰源。我们将问题分解为四个层次：

1. **几何定位**（问题一）：给定多个观测点的示向度，如何确定干扰源的可能位置区域？
2. **最优观测**（问题二）：如何选择第二个观测点使定位精度最高？
3. **全向源搜索**（问题三）：如何高效扫描发现并清除全向干扰源？
4. **混合源检测**（问题四）：定向源存在盲区，如何多角度确认？

---

## 问题一：交会定位区域直径与覆盖性分析

### 数学模型

设观测点 $P_i$ 测得干扰源的示向度为 $\theta_i$（方位角），则干扰源必然位于从 $P_i$ 出发、方向为 $\theta_i$ 的射线上。

由于测角误差 $\pm \epsilon$（本题 $\epsilon = 1°$），实际示向度在 $[\theta_i - \epsilon, \theta_i + \epsilon]$ 范围内，因此干扰源位于一个**楔形区域**（两条射线夹成的扇形）。

多个观测点的楔形区域交集即为干扰源的可能位置区域——一个**凸多边形**。

### 算法实现

#### 1. 半平面构造

对每个观测点 $P_i$ 和示向度 $\theta_i$：
- 左边界射线方向：$\theta_i + \epsilon$
- 右边界射线方向：$\theta_i - \epsilon$
- 构造两个半平面：左侧半平面和右侧半平面

半平面表示为 $(n, d)$，满足 $n \cdot x \leq d$。

#### 2. 凸多边形交集

使用**顶点枚举法**求多个半平面的交集：
- 对每对半平面求交线
- 检查交点是否满足所有半平面约束
- 满足的点即为多边形顶点
- 按极角排序得到有序顶点序列

#### 3. 旋转卡壳求直径

凸多边形直径 = 顶点对之间的最大距离。

朴素算法 $O(n^2)$，旋转卡壳 $O(n)$：
- 维护一对"对踵点"
- 沿多边形边界旋转
- 每次更新最大距离

#### 4. Welzl最小包围圆

随机增量算法，期望时间 $O(n)$：
- 递归构造最小包围圆
- 每次加入一个点，若点在圆外则重新构造
- 基础情况：1/2/3个点确定唯一圆

### 关键代码

```python
# geom_core.py
def build_location_polygon(observers, bearings, error_deg, region_radius):
    """构造定位区域凸多边形"""
    half_planes = []
    for obs, bearing in zip(observers, bearings):
        # 左边界半平面
        left_dir = angle_to_direction(bearing + error_deg)
        left_normal = np.array([-left_dir[1], left_dir[0]])
        d_left = np.dot(left_normal, obs)
        half_planes.append((left_normal, d_left))
        
        # 右边界半平面
        right_dir = angle_to_direction(bearing - error_deg)
        right_normal = np.array([right_dir[1], -right_dir[0]])
        d_right = np.dot(right_normal, obs)
        half_planes.append((-right_normal, -d_right))
    
    return polygon_from_half_planes(half_planes, region_radius)
```

### 覆盖性判断

判断"以直径为直径的圆能否覆盖定位区域"：
- 计算定位区域直径 $D$
- 以直径中点为圆心、$D/2$ 为半径作圆
- 检查所有顶点是否在圆内（距离 $\leq D/2$）

---

## 问题二：第二检测点选择策略

### GDOP理论

**几何稀释度（GDOP, Geometric Dilution of Precision）**衡量观测几何对定位精度的影响。

对两个观测点 $P_1, P_2$ 和目标 $T$：

$$\text{GDOP} = \frac{1}{\sin\theta} \sqrt{\frac{1}{d_1^2} + \frac{1}{d_2^2}}$$

其中：
- $\theta$：两条视线在目标处的夹角（交会角）
- $d_1, d_2$：观测点到目标的距离

**GDOP越小，定位精度越高**。当 $\theta = 90°$ 时 $\sin\theta = 1$ 最大，GDOP最小。

### 优化策略

**目标**：选择第二观测点 $P_2$ 使 GDOP 最小。

**方法**：极坐标网格搜索
1. 以目标估计位置为参考
2. 在 $[0, 2\pi) \times [r_{min}, r_{max}]$ 网格上遍历候选点
3. 对每个候选点计算 GDOP
4. 选择 GDOP 最小的点

**约束**：
- 候选点必须在区域内（$r \leq 1800m$）
- 示向度差不能太大（否则测不到目标）

### 关键代码

```python
# q2_solver.py
def compute_gdop(observer1, observer2, target):
    """计算GDOP"""
    dir1 = target - observer1
    dir2 = target - observer2
    dist1 = np.linalg.norm(dir1)
    dist2 = np.linalg.norm(dir2)
    
    dir1 = dir1 / dist1
    dir2 = dir2 / dist2
    
    # 2维叉积：sin(theta) = |dir1 × dir2|
    sin_theta = abs(dir1[0] * dir2[1] - dir1[1] * dir2[0])
    
    gdop = 1.0 / sin_theta * np.sqrt(1.0 / dist1**2 + 1.0 / dist2**2)
    return gdop
```

### 候选区域特征

最优第二检测点通常位于：
- 与第一观测点-目标连线**近似垂直**的方向
- 距离与目标距离**同量级**
- 使交会角接近 **90°**

---

## 问题三：全向干扰源快速搜索与清除

### 整体策略

**分层搜索**：
1. **粗搜索**：螺旋扫描发现所有活跃频道
2. **交会定位**：用多测量点示向度确定源位置
3. **精定位清除**：接近源后迭代逼近并清除

### 螺旋扫描

生成覆盖路径：
- 5-6个同心圆环，半径递增
- 每环均匀分布检测点
- 边界额外布点（防贴边源）

```python
def _spiral_scan_positions(n_rings=5, points_per_ring=8):
    positions = []
    for ring in range(1, n_rings + 1):
        r = ring * REGION_RADIUS / (n_rings + 1)
        n_pts = points_per_ring + ring * 2
        for i in range(n_pts):
            theta = 2 * np.pi * i / n_pts
            pos = np.array([r * np.cos(theta), r * np.sin(theta)])
            positions.append(pos)
    return positions
```

### 交会定位

对每个频道：
1. 收集所有检测到该频道的测量点
2. 选择交会角最接近90°的测量对
3. 用 `build_location_polygon` 构造定位多边形
4. 取多边形质心作为源位置估计

### 精定位清除

```
循环：
1. 移动到估计位置
2. 测量当前频道
3. 若有信号，沿示向度方向前进一小步（20-30m）
4. 重复直到距离源 < 20m
5. 执行清除
```

**关键**：迭代逼近，不要一次走太远（清除半径仅20m）。

---

## 问题四：混合干扰源检测与清除

### 难点：定向源盲区

定向源只在180°发射半平面内有效，机器狗可能在某些位置收不到信号（不是没源，而是方向不对）。

### 多角度确认机制

1. **初始检测**：螺旋扫描发现信号
2. **多方向复测**：从不同方向接近，确认信号存在
3. **边界扫描**：对无信号区域沿边界扫，排除贴边定向源

### 关键代码

```python
def _multi_angle_confirm(channel, initial_measurements):
    """多角度确认：从多个方向检测，排除定向源盲区误判"""
    # 选择交会角最接近90°的测量对
    best_pair = None
    best_angle_score = -float('inf')
    
    for i in range(len(initial_measurements)):
        for j in range(i + 1, len(initial_measurements)):
            bi = initial_measurements[i]["bearing"]
            bj = initial_measurements[j]["bearing"]
            diff = abs(bi - bj)
            if diff > 180:
                diff = 360 - diff
            score = -abs(diff - 90)  # 越接近90°越好
            if score > best_angle_score:
                best_angle_score = score
                best_pair = (i, j)
    
    # 用最佳测量对定位
    obs = [initial_measurements[best_pair[0]]["position"],
           initial_measurements[best_pair[1]]["position"]]
    bearings = [initial_measurements[best_pair[0]]["bearing"],
                initial_measurements[best_pair[1]]["bearing"]]
    
    polygon = build_location_polygon(obs, bearings, ANGLE_ERROR_DEG, REGION_RADIUS)
    if polygon is not None and len(polygon) >= 3:
        return polygon.mean(axis=0)
    
    # 兆底：取所有测量点质心
    positions = np.array([m["position"] for m in initial_measurements])
    return positions.mean(axis=0)
```

### 边界扫描

沿区域边界（r = 1700m）均匀布点，检测贴边定向源：

```python
def _boundary_sweep(channel):
    """沿边界扫描，检测贴边的定向源"""
    measurements = []
    n_points = 20
    for i in range(n_points):
        theta = 2 * np.pi * i / n_points
        pos = np.array([
            (REGION_RADIUS - 100) * np.cos(theta),
            (REGION_RADIUS - 100) * np.sin(theta),
        ])
        self.sim.move_to(pos)
        result = self.sim.measure(channel)
        if result["measurements"]:
            for src_id, meas in result["measurements"].items():
                measurements.append({
                    "position": pos.copy(),
                    "bearing": meas["bearing"],
                    "source_id": src_id,
                })
    return measurements
```

---

## 本地模拟器设计

### 核心接口

严格按照题目附件1/2协议实现：

```python
class LocalSimulator:
    def enter(self, target_position=None) -> Dict:
        """/enter 指令：进入目标区域"""
    
    def measure(self, channel: int) -> Dict:
        """/measure 指令：测量指定频道的示向度"""
    
    def clear(self) -> Dict:
        """/clear 指令：清除当前位置的干扰源"""
    
    def exit(self) -> Dict:
        """/exit 指令：离开区域"""
    
    def move_to(self, target_position: np.ndarray) -> Dict:
        """移动机器狗到指定位置"""
```

### 计时规则

- 移动：`distance / 5.0` 秒（速度5m/s）
- 切换频道：1秒
- 测量：5秒
- 精确定位：3秒
- 清除：5秒

### 测角误差模型

**题目要求**：同一位置的测角误差固定。

**实现**：用位置+源ID做种子的确定性随机

```python
def _get_angle_error(self, position, source_id):
    """获取测角误差（同位置固定）"""
    pos_key = (round(position[0], 1), round(position[1], 1))
    
    # 缓存检查
    if pos_key in self._position_errors:
        if source_id in self._position_errors[pos_key]:
            return self._position_errors[pos_key][source_id]
    
    # 确定性哈希混合
    x_key = int(round(pos_key[0] * 10))
    y_key = int(round(pos_key[1] * 10))
    raw = (self._global_error_seed
           + x_key * 1000003
           + y_key * 65537
           + source_id * 2654435761)
    seed = raw % (2 ** 32)
    
    rng = np.random.RandomState(seed)
    
    # 缓存
    self._position_errors.setdefault(pos_key, {})[source_id] = error
    return error
```

### 定向源覆盖判断

```python
def _can_receive(self, robot_pos, source):
    """判断机器狗能否收到干扰源信号"""
    if source.cleared:
        return False
    
    distance = np.linalg.norm(robot_pos - source.position)
    if distance > source.receive_radius:
        return False
    
    if source.is_directional:
        # 定向源：检查是否在180°发射半平面内
        direction = np.array([
            np.cos(np.radians(source.direction_angle)),
            np.sin(np.radians(source.direction_angle))
        ])
        to_robot = robot_pos - source.position
        to_robot_norm = to_robot / (np.linalg.norm(to_robot) + 1e-12)
        
        cos_angle = np.dot(direction, to_robot_norm)
        if cos_angle < -0.01:  # 在背面（>90°）
            return False

    return True
```

---

## 关键技术细节

### numpy 2.x 兼容性

`np.cross` 不再支持2维向量，需手写：

```python
def _cross2d(a, b):
    """2维叉积: cross(a,b) = a_x*b_y - a_y*b_x"""
    return a[0] * b[1] - a[1] * b[0]
```

### Windows中文输出

```bash
python -X utf8 main.py
```

### matplotlib中文字体

```python
# viz.py
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "DengXian", "DejaVu Sans"
]
plt.rcParams["axes.unicode_minus"] = False
```

---

## 性能分析

### 时间复杂度

- **问题一**：$O(n^2)$ 顶点枚举 + $O(n)$ 旋转卡壳 + $O(n)$ Welzl = $O(n^2)$
- **问题二**：$O(N_{grid}^2)$ 网格搜索（$N_{grid} = 100$）
- **问题三**：$O(N_{scan} \times N_{channel})$ 扫描 + $O(N_{source} \times N_{iter})$ 清除
- **问题四**：同Q3 + 边界扫描 $O(N_{boundary})$

### 空间复杂度

- **问题一**：$O(n)$ 存储多边形顶点
- **问题二**：$O(N_{grid}^2)$ GDOP地图
- **问题三/Q4**：$O(N_{measurements})$ 存储测量记录

### 虚拟时间消耗

- **Q3**：~18000s（扫描60点×20频道×6s + 移动 + 清除）
- **Q4**：~31000s（更密扫描 + 边界补扫 + 多角度确认）
- **上限**：360000s，远未用满

---

## 改进方向

### 问题三/Q4策略优化

当前完成率偏低（Q3: 6.25%, Q4: 50%），主要瓶颈：

1. **定位精度不够**：只用一对测量点交会，误差大
   - 改进：全部测量点加权最小二乘
   
2. **清除策略粗糙**：一步50m太粗糙
   - 改进：迭代逼近（测→走→测→走，逐步收敛）

3. **扫描密度不足**：5环60点可能漏检
   - 改进：自适应加密（检测到信号的区域增加采样）

### 模拟器优化

1. **信号强度模型**：当前只用距离判断，可加入衰减模型
2. **多源干扰**：当前假设同频道只有一个源，实际可能多个
3. **动态源**：源可能移动或开关

---

## 总结

本题的核心是**几何定位 + 路径规划**：
- 用半平面交集构造定位区域
- 用GDOP理论优化观测点选择
- 用分层搜索策略平衡效率与覆盖率
- 用迭代逼近提高清除成功率

关键技巧：
- 旋转卡壳求直径
- Welzl最小包围圆
- 确定性哈希保证同位置同误差
- numpy 2.x兼容处理

---

**文档版本**：v1.0  
**最后更新**：2026-10-21  
**作者**：flowingyears
