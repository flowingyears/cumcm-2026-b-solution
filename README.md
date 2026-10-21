# 2026年高教社杯全国大学生数学建模竞赛B题

## 无线电干扰源的快速自动定位与清除

本项目是2026年高教社杯全国大学生数学建模竞赛B题的完整解题方案，包含理论分析、算法实现、本地模拟器和本地测试。

## 题目概述

在半径1800m的圆形区域内，有10-16个未知位置的无线电干扰源（全向源与定向源混合），分布在20个频道上。机器狗需要：
- 通过示向度测量定位干扰源
- 在20m范围内清除干扰源
- 最小化虚拟时间消耗

## 四个问题的求解

### 问题一：交会定位区域直径与覆盖性分析
- **核心方法**：半平面构造 + 凸多边形交集 + 旋转卡壳求直径 + Welzl最小包围圆
- **关键输出**：定位区域凸多边形、直径、最小包围圆半径、覆盖性判断
- **实现文件**：`src/q1_solver.py`、`src/geom_core.py`

### 问题二：第二检测点选择策略
- **核心方法**：GDOP（几何稀释度）理论 + 极坐标网格搜索
- **优化准则**：交会角接近90°时定位精度最高
- **实现文件**：`src/q2_solver.py`

### 问题三：全向干扰源快速搜索与清除
- **策略**：螺旋扫描粗搜索 → 交会定位 → 精定位清除
- **实现文件**：`src/q3_solver.py`

### 问题四：混合干扰源检测与清除
- **策略**：多角度确认机制 + 边界扫描（应对定向源盲区）
- **实现文件**：`src/q4_solver.py`

## 项目结构

```
.
├── main.py                 # 主入口，依次求解四个问题
├── requirements.txt        # Python依赖
├── src/
│   ├── config.py          # 全局参数配置
│   ├── geom_core.py       # 几何核心算法（凸多边形、旋转卡壳、最小包围圆）
│   ├── simulator.py       # 本地模拟器（按附件1/2协议实现）
│   ├── q1_solver.py       # 问题一求解器
│   ├── q2_solver.py       # 问题二求解器
│   ├── q3_solver.py       # 问题三求解器
│   ├── q4_solver.py       # 问题四求解器
│   └── viz.py             # matplotlib中文字体配置
├── results/
│   ├── figures/           # 可视化图（q1_result.png, q2_result.png）
│   └── logs/              # 操作日志（q3_log.json, q4_log.json）
├── docs/
│   └── solution.md        # 详细解题思路文档
└── tests/                 # 测试脚本
```

## 运行方法

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 运行完整求解
```bash
python -X utf8 main.py
```

注意：Windows系统需要加 `-X utf8` 参数避免中文输出乱码。

### 3. 单独运行某一问题
```bash
python -X utf8 src/q1_solver.py
python -X utf8 src/q2_solver.py
python -X utf8 src/q3_solver.py
python -X utf8 src/q4_solver.py
```

## 关键算法说明

### 几何核心（geom_core.py）
- **半平面构造**：根据观测点位置和示向度，构造左右边界半平面
- **凸多边形交集**：顶点枚举法求多个半平面的交集
- **旋转卡壳**：O(n)计算凸多边形直径（顶点对最大距离）
- **Welzl算法**：随机增量法求最小包围圆
- **numpy 2.x兼容**：手写 `_cross2d` 替代已弃用的 `np.cross`（2维向量）

### 本地模拟器（simulator.py）
严格按照题目附件1（模拟器使用说明）和附件2（HTTP JSON通信协议）实现：
- 四条指令：`/enter`、`/measure`、`/clear`、`/exit`
- 计时规则：移动5m/s、切换频道1s、测量5s、精确定位3s、清除5s
- 测角误差：同位置固定（用位置+源ID做种子的确定性随机）
- 定向源：180°发射半平面，用余弦判断覆盖

## 输出结果

运行 `main.py` 后：
- **控制台**：打印四问的关键数值结果
- **figures/**：Q1定位区域可视化、Q2候选区域分析图
- **logs/**：Q3/Q4的完整操作日志（JSON格式）

## 技术栈

- Python 3.12+
- NumPy 2.5.4（几何计算）
- SciPy 1.18.1（凸包等）
- Matplotlib 3.11.2（可视化）
- Pandas 3.0.6（日志统计）

## 注意事项

1. **官方模拟器**：本题Q3/Q4最终验证需用组委会官方模拟器（需参赛队号登录获取），本项目使用自建高保真模拟器开发调试
2. **中文字体**：matplotlib默认不支持中文，`src/viz.py`配置了微软雅黑字体回退
3. **虚拟时间**：Q3/Q4的虚拟时间消耗远低于360000s上限，策略优化空间主要在完成率
4. **numpy版本**：代码兼容numpy 2.x（已修复`np.cross`弃用问题）

## 参考资源

- [li2396803/cumcm2026](https://github.com/li2396803/cumcm2026) — ABC三题完整方案
- [ZYAONS/cumcm-2026-b-attempts-and-thoughts](https://github.com/ZYAONS/cumcm-2026-b-attempts-and-thoughts) — 完整研究记录+68项验证套件

## 许可证

本项目代码仅供学习参考。

---

**作者**：flowingyears  
**日期**：2026-10-21  
**竞赛**：2026年高教社杯全国大学生数学建模竞赛