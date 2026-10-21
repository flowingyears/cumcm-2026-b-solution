"""
本地模拟器
按附件2协议实现 /enter /measure /clear /exit 四条指令
用于开发调试，最终结果需用官方模拟器验证
"""

import numpy as np
import json
import time
import os
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from src.config import (
    REGION_RADIUS, NUM_CHANNELS, ROBOT_SPEED,
    CHANNEL_SWITCH_TIME, MEASURE_TIME, PRECISE_LOCATE_TIME,
    CLEAR_TIME, CLEARANCE_RADIUS, ANGLE_ERROR_DEG,
    RECEIVE_RADIUS_MIN, RECEIVE_RADIUS_MAX,
    NUM_SOURCES_MIN, NUM_SOURCES_MAX, VIRTUAL_TIME_LIMIT,
)


@dataclass
class InterferenceSource:
    """干扰源"""
    source_id: int
    position: np.ndarray
    channel: int
    is_directional: bool
    direction_angle: float = 0.0
    receive_radius: float = 1200.0
    cleared: bool = False

    @property
    def coverage_half_planes(self) -> List[Tuple[np.ndarray, float]]:
        if not self.is_directional:
            return []
        direction = np.array([
            np.cos(np.radians(self.direction_angle)),
            np.sin(np.radians(self.direction_angle))
        ])
        normal = np.array([-direction[1], direction[0]])
        d = normal @ self.position
        return [
            (normal, d + 1e-6),
            (-normal, -d + 1e-6),
        ]


@dataclass
class RobotState:
    """机器狗状态"""
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    current_channel: int = 0
    is_inside: bool = False
    virtual_time: float = 0.0
    action_log: List[Dict] = field(default_factory=list)


class LocalSimulator:
    """本地模拟器"""

    def __init__(
        self,
        num_sources: Optional[int] = None,
        seed: Optional[int] = None,
        fixed_errors: Optional[Dict[Tuple[float, float], Dict[int, float]]] = None,
    ):
        if seed is not None:
            np.random.seed(seed)

        if num_sources is None:
            num_sources = np.random.randint(NUM_SOURCES_MIN, NUM_SOURCES_MAX + 1)

        self.num_sources = num_sources
        self.sources: List[InterferenceSource] = []
        self.robot = RobotState()
        self._generate_sources()

        self._position_errors = fixed_errors or {}
        self._global_error_seed = seed if seed is not None else int(time.time())

    def _generate_sources(self):
        """随机生成干扰源"""
        channels = np.random.choice(NUM_CHANNELS, self.num_sources, replace=False)

        for i in range(self.num_sources):
            while True:
                angle = np.random.uniform(0, 2 * np.pi)
                r = np.random.uniform(0, REGION_RADIUS * 0.95)
                pos = np.array([r * np.cos(angle), r * np.sin(angle)])
                if np.linalg.norm(pos) <= REGION_RADIUS:
                    break

            is_directional = np.random.random() < 0.3
            direction_angle = np.random.uniform(0, 360)
            receive_radius = np.random.uniform(RECEIVE_RADIUS_MIN, RECEIVE_RADIUS_MAX)

            source = InterferenceSource(
                source_id=i,
                position=pos,
                channel=int(channels[i]),
                is_directional=is_directional,
                direction_angle=direction_angle,
                receive_radius=receive_radius,
            )
            self.sources.append(source)

    def _get_angle_error(self, position: np.ndarray, source_id: int) -> float:
        """获取测角误差"""
        pos_key = (round(position[0], 1), round(position[1], 1))

        if pos_key in self._position_errors:
            if source_id in self._position_errors[pos_key]:
                return self._position_errors[pos_key][source_id]

        # 确定性哈希混合：同位置+同源 → 永远同一误差（题目要求误差同位置固定）。
        # 先把位置取整到分米级整数键，消除浮点噪声；再混合取模，
        # 保证种子始终落在 RandomState 要求的 [0, 2**32-1] 区间（负坐标不再溢出）。
        x_key = int(round(pos_key[0] * 10))
        y_key = int(round(pos_key[1] * 10))
        raw = (self._global_error_seed
               + x_key * 1000003
               + y_key * 65537
               + source_id * 2654435761)
        seed = raw % (2 ** 32)

        rng = np.random.RandomState(seed)
        return rng.uniform(-ANGLE_ERROR_DEG, ANGLE_ERROR_DEG)

    def _can_receive(self, robot_pos: np.ndarray, source: InterferenceSource) -> bool:
        """判断机器狗能否收到干扰源信号"""
        if source.cleared:
            return False

        distance = np.linalg.norm(robot_pos - source.position)
        if distance > source.receive_radius:
            return False

        if source.is_directional:
            direction = np.array([
                np.cos(np.radians(source.direction_angle)),
                np.sin(np.radians(source.direction_angle))
            ])
            to_robot = robot_pos - source.position
            to_robot_norm = to_robot / (np.linalg.norm(to_robot) + 1e-12)

            cos_angle = np.dot(direction, to_robot_norm)
            if cos_angle < -0.01:
                return False

        return True

    def _compute_bearing(
        self, robot_pos: np.ndarray, source: InterferenceSource
    ) -> Optional[float]:
        """计算机器狗测得的干扰源示向度"""
        if not self._can_receive(robot_pos, source):
            return None

        true_direction = source.position - robot_pos
        true_bearing = np.degrees(np.arctan2(true_direction[1], true_direction[0]))

        error = self._get_angle_error(robot_pos, source.source_id)
        measured_bearing = (true_bearing + error) % 360

        return measured_bearing

    def _move_cost(self, from_pos: np.ndarray, to_pos: np.ndarray) -> float:
        """计算移动耗时"""
        distance = np.linalg.norm(to_pos - from_pos)
        return distance / ROBOT_SPEED

    def _channel_switch_cost(self, from_channel: int, to_channel: int) -> float:
        """计算频道切换耗时"""
        if from_channel == to_channel:
            return 0.0
        return CHANNEL_SWITCH_TIME

    def enter(self, target_position: Optional[np.ndarray] = None) -> Dict:
        """/enter 指令：进入目标区域"""
        if self.robot.is_inside:
            return {"status": "error", "message": "Already inside"}

        if target_position is not None:
            move_time = self._move_cost(self.robot.position, target_position)
            self.robot.virtual_time += move_time
            self.robot.position = np.array(target_position, dtype=float)

        self.robot.is_inside = True

        log_entry = {
            "action": "enter",
            "position": self.robot.position.tolist(),
            "virtual_time": self.robot.virtual_time,
        }
        self.robot.action_log.append(log_entry)

        return {
            "status": "ok",
            "position": self.robot.position.tolist(),
            "virtual_time": self.robot.virtual_time,
        }

    def measure(self, channel: int) -> Dict:
        """/measure 指令：在当前位置测量指定频道的示向度"""
        if not self.robot.is_inside:
            return {"status": "error", "message": "Not inside, call /enter first"}

        switch_time = self._channel_switch_cost(self.robot.current_channel, channel)
        self.robot.virtual_time += switch_time
        self.robot.current_channel = channel

        self.robot.virtual_time += MEASURE_TIME

        bearings = {}
        for source in self.sources:
            if source.channel != channel:
                continue
            bearing = self._compute_bearing(self.robot.position, source)
            if bearing is not None:
                bearings[source.source_id] = {
                    "bearing": bearing,
                    "signal_strength": self._compute_signal_strength(source),
                }

        log_entry = {
            "action": "measure",
            "position": self.robot.position.tolist(),
            "channel": channel,
            "bearings": bearings,
            "virtual_time": self.robot.virtual_time,
        }
        self.robot.action_log.append(log_entry)

        return {
            "status": "ok",
            "position": self.robot.position.tolist(),
            "channel": channel,
            "measurements": bearings,
            "virtual_time": self.robot.virtual_time,
        }

    def move_to(self, target_position: np.ndarray) -> Dict:
        """移动机器狗到指定位置"""
        if not self.robot.is_inside:
            return {"status": "error", "message": "Not inside"}

        target = np.array(target_position, dtype=float)
        distance = np.linalg.norm(target - self.robot.position)

        if distance > REGION_RADIUS * 2:
            return {"status": "error", "message": "Target out of bounds"}

        move_time = self._move_cost(self.robot.position, target)
        self.robot.virtual_time += move_time
        self.robot.position = target

        return {
            "status": "ok",
            "position": self.robot.position.tolist(),
            "virtual_time": self.robot.virtual_time,
        }

    def clear(self) -> Dict:
        """/clear 指令：清除当前位置的干扰源"""
        if not self.robot.is_inside:
            return {"status": "error", "message": "Not inside"}

        self.robot.virtual_time += PRECISE_LOCATE_TIME + CLEAR_TIME

        cleared_sources = []
        for source in self.sources:
            if source.cleared:
                continue
            distance = np.linalg.norm(self.robot.position - source.position)
            if distance <= CLEARANCE_RADIUS:
                source.cleared = True
                cleared_sources.append(source.source_id)

        log_entry = {
            "action": "clear",
            "position": self.robot.position.tolist(),
            "cleared_sources": cleared_sources,
            "virtual_time": self.robot.virtual_time,
        }
        self.robot.action_log.append(log_entry)

        return {
            "status": "ok",
            "cleared": cleared_sources,
            "virtual_time": self.robot.virtual_time,
        }

    def exit(self) -> Dict:
        """/exit 指令：离开区域"""
        if not self.robot.is_inside:
            return {"status": "error", "message": "Not inside"}

        self.robot.is_inside = False

        log_entry = {
            "action": "exit",
            "virtual_time": self.robot.virtual_time,
            "total_cleared": sum(1 for s in self.sources if s.cleared),
            "total_sources": len(self.sources),
        }
        self.robot.action_log.append(log_entry)

        return {
            "status": "ok",
            "virtual_time": self.robot.virtual_time,
            "total_cleared": log_entry["total_cleared"],
            "total_sources": log_entry["total_sources"],
        }

    def _compute_signal_strength(self, source: InterferenceSource) -> float:
        """计算信号强度（仅与距离有关）"""
        distance = np.linalg.norm(self.robot.position - source.position)
        if distance < 1.0:
            distance = 1.0
        return 1.0 / (distance ** 2)

    def get_state(self) -> Dict:
        """获取当前状态"""
        return {
            "robot_position": self.robot.position.tolist(),
            "robot_channel": self.robot.current_channel,
            "robot_inside": self.robot.is_inside,
            "virtual_time": self.robot.virtual_time,
            "sources_total": len(self.sources),
            "sources_cleared": sum(1 for s in self.sources if s.cleared),
            "sources_detail": [
                {
                    "id": s.source_id,
                    "channel": s.channel,
                    "cleared": s.cleared,
                    "is_directional": s.is_directional,
                }
                for s in self.sources
            ],
        }

    def reset(self):
        """重置模拟器"""
        self.robot = RobotState()
        for source in self.sources:
            source.cleared = False

    def export_log(self, filepath: str):
        """导出操作日志"""
        log_data = {
            "num_sources": self.num_sources,
            "sources": [
                {
                    "id": s.source_id,
                    "position": s.position.tolist(),
                    "channel": s.channel,
                    "is_directional": s.is_directional,
                    "direction_angle": s.direction_angle,
                    "receive_radius": s.receive_radius,
                }
                for s in self.sources
            ],
            "actions": self.robot.action_log,
            "final_virtual_time": self.robot.virtual_time,
        }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(log_data, f, ensure_ascii=False, indent=2)

    def get_all_active_channels(self) -> List[int]:
        """获取所有未清除干扰源的频道"""
        return [s.channel for s in self.sources if not s.cleared]

    def get_sources_on_channel(self, channel: int) -> List[InterferenceSource]:
        """获取指定频道上的干扰源"""
        return [s for s in self.sources if s.channel == channel]
