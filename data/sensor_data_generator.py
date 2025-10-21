#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器数据生成器
根据sensor数据结构生成txt数据文件
包含传感器ID、坐标(x,y,z)、能量消耗数据
"""

import numpy as np
import random
import os
from utils.utils import Point
from utils.parameters import InputParameter

class SensorDataGenerator:
    """传感器数据生成器类"""
    
    def __init__(self, sensor_num=50, area_long=200, area_wide=100):
        """
        初始化传感器数据生成器
        
        Args:
            sensor_num: 传感器数量
            area_long: 区域长度
            area_wide: 区域宽度
        """
        self.sensor_num = sensor_num
        self.area_long = area_long
        self.area_wide = area_wide
        
        # 正态分布参数设置（能量消耗范围100-1000）
        self.energy_mean = 550  # 均值
        self.energy_std = 150   # 标准差，确保大部分数据在100-1000范围内
        
    def generate_sensor_coordinates(self):
        """
        生成传感器坐标
        x, y在区域内随机分布，z在[0,50]范围内随机分布
        
        Returns:
            list: 包含所有传感器坐标的列表
        """
        coordinates = []
        
        for i in range(self.sensor_num):
            # x坐标：在[0, area_long]范围内随机生成
            x = round(random.uniform(0, self.area_long), 2)
            
            # y坐标：在[0, area_wide]范围内随机生成
            y = round(random.uniform(0, self.area_wide), 2)
            
            # z坐标：在[0, 50]范围内随机生成
            z = round(random.uniform(0, 50), 2)
            
            coordinates.append(Point(x, y, z))
            
        return coordinates
    
    def generate_energy_consumption(self):
        """
        生成能量消耗数据（正态分布）
        范围在(100, 1000)内
        
        Returns:
            list: 包含所有传感器能量消耗的列表
        """
        energy_consumptions = []
        
        for i in range(self.sensor_num):
            # 生成正态分布的能量消耗数据
            energy = np.random.normal(self.energy_mean, self.energy_std)
            
            # 确保数据在(100, 1000)范围内
            energy = max(100, min(1000, energy))
            
            # 保留2位小数
            energy = round(energy, 2)
            
            energy_consumptions.append(energy)
            
        return energy_consumptions
    
    def generate_sensor_data(self):
        """
        生成完整的传感器数据
        
        Returns:
            list: 包含所有传感器数据的列表，每个元素为(sensor_id, x, y, z, energy)
        """
        coordinates = self.generate_sensor_coordinates()
        energy_consumptions = self.generate_energy_consumption()
        
        sensor_data = []
        
        for i in range(self.sensor_num):
            sensor_id = i + 1  # 传感器ID从1开始
            coord = coordinates[i]
            energy = energy_consumptions[i]
            
            sensor_data.append((
                sensor_id,
                coord.x,
                coord.y,
                coord.z,
                energy
            ))
            
        return sensor_data
    
    def save_to_txt(self, sensor_data, filename="sensor_data.txt"):
        """
        将传感器数据保存到txt文件
        
        Args:
            sensor_data: 传感器数据列表
            filename: 输出文件名
        """
        with open(filename, 'w', encoding='utf-8') as f:
            # 直接写入数据（使用固定宽度格式对齐）
            for data in sensor_data:
                sensor_id, x, y, z, energy = data
                f.write(f"{sensor_id:<2} {x:>9.2f} {y:>9.2f} {z:>9.2f} {energy:>9.2f}\n")
        
        print(f"传感器数据已保存到 {filename}")
        print(f"共生成 {len(sensor_data)} 个传感器的数据")
    
    def print_statistics(self, sensor_data):
        """
        打印数据统计信息
        
        Args:
            sensor_data: 传感器数据列表
        """
        energies = [data[4] for data in sensor_data]
        x_coords = [data[1] for data in sensor_data]
        y_coords = [data[2] for data in sensor_data]
        z_coords = [data[3] for data in sensor_data]
        
        print("\n=== 数据统计信息 ===")
        print(f"传感器总数: {len(sensor_data)}")
        print(f"X坐标范围: [{min(x_coords):.2f}, {max(x_coords):.2f}]")
        print(f"Y坐标范围: [{min(y_coords):.2f}, {max(y_coords):.2f}]")
        print(f"Z坐标范围: [{min(z_coords):.2f}, {max(z_coords):.2f}]")
        print(f"能量消耗范围: [{min(energies):.2f}, {max(energies):.2f}]")
        print(f"能量消耗均值: {np.mean(energies):.2f}")
        print(f"能量消耗标准差: {np.std(energies):.2f}")

def main():
    """主函数"""
    try:
        print("开始执行传感器数据生成器...")
        
        # 从parameters.py获取参数
        print("正在加载参数...")
        params = InputParameter()
        print(f"传感器数量: {params.sensor_num}")
        print(f"区域大小: {params.AREA_LONG} x {params.AREA_WIDE}")
        
        # 创建传感器数据生成器
        print("正在创建数据生成器...")
        generator = SensorDataGenerator(
            sensor_num=params.sensor_num,
            area_long=params.AREA_LONG,
            area_wide=params.AREA_WIDE
        )
        
        # 生成传感器数据
        print("正在生成传感器数据...")
        sensor_data = generator.generate_sensor_data()
        
        # 保存到txt文件
        print("正在保存数据到文件...")
        generator.save_to_txt(sensor_data, "sensor_data.txt")
        
        # 打印统计信息
        generator.print_statistics(sensor_data)
        
        # 显示前5条数据作为示例
        print("\n=== 前5条数据示例 ===")
        for i in range(min(5, len(sensor_data))):
            data = sensor_data[i]
            print(f"{data[0]:<8} {data[1]:>9.2f} {data[2]:>9.2f} {data[3]:>9.2f} {data[4]:>9.2f}")
            
        print("程序执行完成！")
        
    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()