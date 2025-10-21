from utils import *
from network import WRSNNetwork

def main():
    """主函数，运行无人机传感器网络系统"""
    # 创建系统参数
    params = InputParameter()

    # 创建网络系统
    network = WRSNNetwork(params)

    # 随机分配无人机对应的传感器编号，将传感器分组
    network._assign_sensors_to_uavs()

    # 演示hover_time计算功能
    print("\n=== 悬停时间计算演示 ===")
    for uav_id in range(network.num_uavs):
        assigned_sensors = network.uav_sensor_assignments.get(uav_id, [])
        if assigned_sensors:
            print(f"\n无人机{uav_id}的悬停时间分析:")
            total_hover_time = network.get_uav_total_hover_time(uav_id)
            print(f"  总悬停时间: {total_hover_time:.2f}秒")

            # 显示每个传感器的悬停时间
            for sensor_id in assigned_sensors[:3]:  # 只显示前3个传感器
                hover_time = network.get_uav_hover_time(uav_id, sensor_id)
                print(f"  传感器{sensor_id}悬停时间: {hover_time:.2f}秒")

    # 运行系统
    runtime = network.run_system()

    # 输出系统状态
    status = network.get_system_status()
    print("\n=== 系统最终状态 ===")
    print(f"数据收集周期:{status['cycle_num']}")
    print(f"系统运行时长: {runtime:.2f}秒")
    print(f"活跃传感器数量: {status['active_sensors']}/{status['total_sensors']}")
    print(f"活跃无人机数量: {status['active_uavs']}/{status['total_uavs']}")
    # print(f"收集的数据总量: {status['collected_data_count']}")
    print(f"终止原因: {status['termination_reason']}")


if __name__ == '__main__':
    main()