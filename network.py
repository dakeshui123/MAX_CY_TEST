from __future__ import annotations

from queue import PriorityQueue

import numpy as np
import enum
import math
import random
from typing import List, Tuple, Dict

# from utils import WrsnParameters
# from utils import NetworkInput, Point, logger
from utils import *
from uav import UAV
from sensor import Sensor, NodeType
from Algorithms.test import plan_uav_path



class WRSNNetwork():
    """无人机无线传感器网络系统
    实现无人机从基站出发，收集传感器数据并充电的系统
    """

    def __init__(self, params: InputParameter = None):
        """
        初始化无人机传感器网络系统
        
        Args:
            params: 系统参数，包含无人机数量、电量、基站坐标等信息
        """
        if params is None:
            params = InputParameter()
        
        self.params = params
        self.num_sensors = params.sensor_num
        self.num_uavs = params.uav_num
        
        # 初始化基站
        self.base_station = params.base_station
        
        # 初始化传感器（从sensor_data.txt文件加载）
        self.sensors = self._load_sensors()
        
        # 初始化无人机
        self.uavs = self._initialize_uavs()
        
        # 系统状态
        self.system_time = 0.0
        self.data_collection_cycle = 60.0  # 数据收集周期（秒）
        self.cycle_start_time = 0.0
        self.cycle_num = 0
        self.collected_data = {}  # 存储收集到的数据
        self.system_terminated = False
        self.termination_reason = ""
        
        # 传感器数据生成参数
        self.data_generation_interval = 10.0  # 传感器每10秒生成一次数据
        self.last_data_generation = {}  # 记录每个传感器上次生成数据的时间
        
        # 初始化传感器数据生成时间
        for sensor in self.sensors:
            self.last_data_generation[sensor.id] = 0.0
            self.collected_data[sensor.id] = []
        
        # 无人机传感器分配
        self.uav_sensor_assignments = {}  # 存储每架无人机负责的传感器ID列表
        self.sensor_uav_mapping = {}  # 存储每个传感器对应的无人机ID

    def _load_sensors(self) -> List[Sensor]:
        """从sensor_data.txt文件加载传感器数据"""
        sensors = []
        try:
            with open('./data/sensor_data.txt', 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    
                    parts = line.split()
                    if len(parts) != 5:
                        print(f"警告：第{line_num}行数据格式不正确，跳过")
                        continue
                    
                    try:
                        sensor_id = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        energy_consumption = float(parts[4])
                        position = Point(x, y, z)
                        sensor = Sensor(position, self.params.SENSOR_POWER, sensor_id)
                        sensor.energy_consumption_rate = energy_consumption
                        sensors.append(sensor)
                        
                    except ValueError as e:
                        print(f"警告：第{line_num}行数据解析错误：{e}，跳过")
                        continue
                        
        except FileNotFoundError:
            print("错误：找不到./data/sensor_data.txt文件，请先运行sensor_data_generator.py生成数据")
            return []
        
        print(f"成功加载{len(sensors)}个传感器")
        return sensors
    
    def _initialize_uavs(self) -> List[UAV]:
        """初始化无人机"""
        uavs = []
        for i in range(self.num_uavs):
            uav = UAV(
                vel=self.params.UAV_SPEED,
                max_E=self.params.UAV_POWER,
                pos=self.base_station
            )
            uav.id = i
            uav.computePower()  # 计算功率
            uavs.append(uav)
        
        print(f"成功初始化{len(uavs)}架无人机")
        return uavs
    
    def _assign_sensors_to_uavs(self):
        """随机分配传感器给无人机，将传感器分组"""
        if not self.sensors:
            print("警告：没有传感器可分配")
            return
        
        # 获取所有传感器ID
        sensor_ids = [sensor.id for sensor in self.sensors]
        
        # 随机打乱传感器ID列表
        random.shuffle(sensor_ids)
        
        # 计算每架无人机应该负责的传感器数量
        sensors_per_uav = len(sensor_ids) // self.num_uavs
        remaining_sensors = len(sensor_ids) % self.num_uavs
        
        # 为每架无人机分配传感器
        start_idx = 0
        for uav_id in range(self.num_uavs):
            # 计算当前无人机负责的传感器数量
            current_sensor_count = sensors_per_uav
            if uav_id < remaining_sensors:  # 前几架无人机多分配一个传感器
                current_sensor_count += 1
            
            # 分配传感器ID
            end_idx = start_idx + current_sensor_count
            assigned_sensors = sensor_ids[start_idx:end_idx]
            
            self.uav_sensor_assignments[uav_id] = assigned_sensors
            
            # 建立传感器到无人机的映射
            for sensor_id in assigned_sensors:
                self.sensor_uav_mapping[sensor_id] = uav_id
            
            start_idx = end_idx
        
        # 输出分配结果
        print("\n=== 传感器分配结果 ===")
        for uav_id in range(self.num_uavs):
            assigned_sensors = self.uav_sensor_assignments[uav_id]
            print(f"无人机{uav_id}负责传感器: {assigned_sensors}")
        
        print(f"\n传感器总数: {len(sensor_ids)}")
        print(f"无人机总数: {self.num_uavs}")
        print(f"平均每架无人机负责: {len(sensor_ids) / self.num_uavs:.1f}个传感器")


    def _plan_uav_path(self, uav: UAV) -> List[Point]:
        """为无人机规划路径，调用test.py中的路径规划算法"""
        return plan_uav_path(uav, self.sensors, self.uav_sensor_assignments)
    
    def _calculate_flight_energy(self, uav: UAV, start_pos: Point, end_pos: Point) -> float:
        """计算无人机飞行能耗"""
        distance = dist(start_pos, end_pos)
        # print(start_pos, end_pos)
        # print(distance)
        flight_time = distance / uav.vel
        return uav.P_mov * flight_time
    
    def _calculate_hover_energy(self, uav: UAV, hover_time: float) -> float:
        """计算无人机悬停能耗"""
        return uav.P_hov * hover_time
    
    def _calculate_charging_energy(self, uav: UAV, sensor: Sensor) -> float:
        """计算无人机充电能耗"""
        # 计算需要充电的能量
        energy_needed = sensor.battery_cap - sensor.cur_energy
        if energy_needed <= 0:
            return 0
        
        # 计算充电时间
        charging_time = energy_needed / (uav.P_tra * 0.9)  # 假设90%效率
        return uav.P_tra * charging_time
    
    def _calculate_hover_time(self, uav: UAV, sensor: Sensor, distance: float = 1.0) -> float:
        """
        计算无人机悬停时间
        
        Args:
            uav: 无人机对象
            sensor: 传感器对象
            distance: 无人机与传感器的距离(m)
            
        Returns:
            float: 悬停时间(秒)
        """
        # 计算需要充电的能量
        energy_needed = sensor.battery_cap - sensor.cur_energy
        if energy_needed <= 0:
            return 0.0
        
        # 使用UAV的computeDataTransTime方法计算传输时间
        try:
            hover_time = uav.computeDataTransTime(distance=distance, E_need=energy_needed)
            return hover_time
        except Exception as e:
            print(f"计算悬停时间时出错: {e}")
            # 备用计算方法
            charging_time = energy_needed / (uav.P_tra * 0.9)  # 假设90%效率
            return charging_time
    
    def _calculate_total_hover_time(self, uav: UAV, sensors: List[Sensor]) -> float:
        """
        计算无人机访问多个传感器的总悬停时间
        
        Args:
            uav: 无人机对象
            sensors: 传感器列表
            
        Returns:
            float: 总悬停时间(秒)
        """
        total_hover_time = 0.0
        
        for sensor in sensors:
            if not sensor.is_active:
                continue
                
            # 计算到传感器的距离（假设无人机在传感器上方1米）
            distance = 1.0  # 无人机悬停在传感器上方1米处
            
            # 计算悬停时间
            hover_time = self._calculate_hover_time(uav, sensor, distance)
            total_hover_time += hover_time
            
            print(f"传感器{sensor.id}悬停时间: {hover_time:.2f}秒")
        
        return total_hover_time
    
    def _simulate_uav_mission(self, uav: UAV) -> bool:
        """模拟无人机执行任务"""
        # 规划路径
        path = self._plan_uav_path(uav)
        if not path:
            return True  # 没有任务可执行
        
        current_pos = uav.pos
        total_energy_needed = 0
        
        # 计算总能耗
        for target_pos in path:
            # 飞行能耗
            flight_energy = self._calculate_flight_energy(uav, current_pos, target_pos)
            total_energy_needed += flight_energy
            
            # 悬停和充电能耗
            sensor_id = None
            for sensor in self.sensors:
                if (abs(sensor.position.x - target_pos.x) < 0.1 and 
                    abs(sensor.position.y - target_pos.y) < 0.1 and
                    abs(sensor.position.z - target_pos.z + 1) < 0.1):
                    sensor_id = sensor.id
                    break
            
            if sensor_id:
                sensor = next(s for s in self.sensors if s.id == sensor_id)
                charging_energy = self._calculate_charging_energy(uav, sensor)
                hover_time = self._calculate_hover_time(uav, sensor, distance=1.0)
                hover_energy = self._calculate_hover_energy(uav, hover_time)
                total_energy_needed += charging_energy + hover_energy
            
            current_pos = target_pos
        
        # 返回基站的能耗
        return_energy = self._calculate_flight_energy(uav, current_pos, self.base_station)
        total_energy_needed += return_energy
        
        # 检查是否有足够电量
        if total_energy_needed > uav.curr_E:
            print(f"无人机{uav.id}电量不足，无法完成任务")
            return False
        
        # 执行任务
        uav.curr_E -= total_energy_needed
        uav.pos = self.base_station  # 返回基站
        
        # 收集数据
        for target_pos in path:
            sensor_id = None
            for sensor in self.sensors:
                if (abs(sensor.position.x - target_pos.x) < 0.1 and 
                    abs(sensor.position.y - target_pos.y) < 0.1 and
                    abs(sensor.position.z - target_pos.z + 1) < 0.1):
                    sensor_id = sensor.id
                break

            if sensor_id:
                sensor = next(s for s in self.sensors if s.id == sensor_id)
                # 收集数据
                if sensor_id in self.collected_data:
                    # 清空已收集的数据
                    self.collected_data[sensor_id] = []
                
                # 给传感器充电
                sensor.cur_energy = sensor.battery_cap
        
        return True
    
    def run_system(self) -> float:
        """运行系统，返回系统运行时长"""
        print("开始运行无人机传感器网络系统...")
        
        while not self.system_terminated:
            
            # 检查是否到了数据收集周期
            if self.system_time - self.cycle_start_time >= self.data_collection_cycle:
                cycle_num = int(self.system_time // self.data_collection_cycle) + 1
                print(f"开始第{cycle_num}个数据收集周期")
                
                # 为每架无人机分配任务
                for uav in self.uavs:
                    if uav.curr_E <= 0:
                        print(f"无人机{uav.id}电量耗尽")
                        self.system_terminated = True
                        self.termination_reason = f"无人机{uav.id}电量耗尽"
                        break
                    
                    # 获取分配给当前无人机的传感器
                    assigned_sensors = self.uav_sensor_assignments.get(uav.id, [])
                    if not assigned_sensors:
                        print(f"无人机{uav.id}没有分配传感器，跳过")
                        continue
                    
                    print(f"无人机{uav.id}开始执行任务，负责传感器: {assigned_sensors}")
                    
                    # 执行任务
                    if not self._simulate_uav_mission(uav):
                        self.system_terminated = True
                        self.termination_reason = "无人机电量不足"
                        break
                    
                    print(f"无人机{uav.id}任务完成，剩余电量: {uav.curr_E:.2f}J")
                
                if not self.system_terminated:
                    self.cycle_start_time = self.system_time
                    self.cycle_num = cycle_num
                    print(f"第{cycle_num}个周期完成")
            
            # 时间推进
            self.system_time += 1.0
        
        print(f"系统终止，原因：{self.termination_reason}")
        print(f"系统总运行时长：{self.system_time:.2f}秒")
        return self.system_time
    
    def get_system_status(self) -> Dict:
        """获取系统状态"""
        active_sensors = sum(1 for s in self.sensors if s.is_active)
        active_uavs = sum(1 for u in self.uavs if u.curr_E > 0)
        
        return {
            'cycle_num': self.cycle_num,
            'system_time': self.system_time,
            'active_sensors': active_sensors,
            'total_sensors': len(self.sensors),
            'active_uavs': active_uavs,
            'total_uavs': len(self.uavs),
            'terminated': self.system_terminated,
            'termination_reason': self.termination_reason,
            'collected_data_count': sum(len(data) for data in self.collected_data.values())
        }
    
    def get_uav_hover_time(self, uav_id: int, sensor_id: int, distance: float = 1.0) -> float:
        """
        获取指定无人机对指定传感器的悬停时间
        
        Args:
            uav_id: 无人机ID
            sensor_id: 传感器ID
            distance: 无人机与传感器的距离(m)
            
        Returns:
            float: 悬停时间(秒)
        """
        if uav_id >= len(self.uavs) or uav_id < 0:
            print(f"错误：无人机ID {uav_id} 不存在")
            return 0.0
        
        sensor = next((s for s in self.sensors if s.id == sensor_id), None)
        if not sensor:
            print(f"错误：传感器ID {sensor_id} 不存在")
            return 0.0
        
        uav = self.uavs[uav_id]
        return self._calculate_hover_time(uav, sensor, distance)
    
    def get_uav_total_hover_time(self, uav_id: int) -> float:
        """
        获取指定无人机的总悬停时间（访问所有分配的传感器）
        
        Args:
            uav_id: 无人机ID
            
        Returns:
            float: 总悬停时间(秒)
        """
        if uav_id >= len(self.uavs) or uav_id < 0:
            print(f"错误：无人机ID {uav_id} 不存在")
            return 0.0
        
        uav = self.uavs[uav_id]
        assigned_sensor_ids = self.uav_sensor_assignments.get(uav_id, [])
        assigned_sensors = [s for s in self.sensors if s.id in assigned_sensor_ids and s.is_active]
        
        return self._calculate_total_hover_time(uav, assigned_sensors)



