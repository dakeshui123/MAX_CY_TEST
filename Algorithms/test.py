from typing import List
from utils import Point, dist
from uav import UAV
from sensor import Sensor


class PathPlanner:
    """路径规划算法类"""
    
    def __init__(self):
        pass
    
    def plan_uav_path(self, uav: UAV, sensors: List[Sensor], uav_sensor_assignments: dict) -> List[Point]:
        """
        为无人机规划路径（简单的贪心算法），只访问分配给该无人机的传感器
        
        Args:
            uav: 无人机对象
            sensors: 所有传感器列表
            uav_sensor_assignments: 无人机传感器分配字典
            
        Returns:
            List[Point]: 规划好的路径点列表
        """
        if not sensors:
            return []
        
        # 获取分配给当前无人机的传感器ID
        assigned_sensor_ids = uav_sensor_assignments.get(uav.id, [])
        if not assigned_sensor_ids:
            return []
        
        # 获取分配给当前无人机的活跃传感器
        assigned_sensors = [s for s in sensors if s.id in assigned_sensor_ids and s.is_active]
        if not assigned_sensors:
            return []
        
        # 简单的贪心算法：选择距离当前位置最近的传感器
        current_pos = uav.pos
        path = []
        remaining_sensors = assigned_sensors.copy()
        
        while remaining_sensors:
            # 找到距离当前位置最近的传感器
            min_dist = float('inf')
            next_sensor = None
            
            for sensor in remaining_sensors:
                dist_to_sensor = dist(current_pos, sensor.position)
                if dist_to_sensor < min_dist:
                    min_dist = dist_to_sensor
                    next_sensor = sensor
            
            if next_sensor:
                # 无人机飞到传感器上方1米处
                target_pos = Point(next_sensor.position.x, next_sensor.position.y, next_sensor.position.z + 1)
                path.append(target_pos)
                current_pos = target_pos
                remaining_sensors.remove(next_sensor)
        
        return path


# 创建全局路径规划器实例
path_planner = PathPlanner()


def plan_uav_path(uav: UAV, sensors: List[Sensor], uav_sensor_assignments: dict) -> List[Point]:
    """
    为无人机规划路径的全局函数
    
    Args:
        uav: 无人机对象
        sensors: 所有传感器列表
        uav_sensor_assignments: 无人机传感器分配字典
        
    Returns:
        List[Point]: 规划好的路径点列表
    """
    return path_planner.plan_uav_path(uav, sensors, uav_sensor_assignments)