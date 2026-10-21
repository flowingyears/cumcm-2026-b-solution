"""
matplotlib 全局绘图样式：中文字体配置
所有画图的模块 import 本模块即可让中文正常显示（不再变方块）
"""

import matplotlib.pyplot as plt

# Windows 自带的中文字体，按优先级回退：
# 微软雅黑 > 黑体 > 无（最后的 DejaVu Sans 兜底显示英文和数字）
plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "DengXian", "DejaVu Sans"
]
# 解决负号 '-' 显示为方块的问题
plt.rcParams["axes.unicode_minus"] = False