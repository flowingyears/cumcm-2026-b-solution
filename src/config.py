"""
2026年高教社杯全国大学生数学建模竞赛 B题
无线电干扰源的快速自动定位与清除
全局参数配置
"""

import os

# 项目根目录（src 的上一级），保证输出建在项目根而不是 src/ 里
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "data")
# 所有输出统一收进 results/ 一个文件夹：results/figures 放图，results/logs 放日志
RESULTS_DIR = os.path.join(BASE_DIR, "results")
FIGURES_DIR = os.path.join(RESULTS_DIR, "figures")
LOGS_DIR = os.path.join(RESULTS_DIR, "logs")

REGION_RADIUS = 1800.0

NUM_CHANNELS = 20
NUM_SOURCES_MIN = 10
NUM_SOURCES_MAX = 16

ROBOT_SPEED = 5.0
CHANNEL_SWITCH_TIME = 1.0
MEASURE_TIME = 5.0
PRECISE_LOCATE_TIME = 3.0
CLEAR_TIME = 5.0
CLEARANCE_RADIUS = 20.0
NEAR_THRESHOLD = 5.0

ANGLE_ERROR_DEG = 1.0

RECEIVE_RADIUS_MIN = 1000.0
RECEIVE_RADIUS_MAX = 1500.0

VIRTUAL_TIME_LIMIT = 360000.0
COORD_LIMIT = 2e6

PROGRAM_TIME_LIMIT = 20 * 60
TEST_WINDOW = 25 * 60
NUM_FORMAL_TESTS = 3

ORIGIN = (0.0, 0.0)