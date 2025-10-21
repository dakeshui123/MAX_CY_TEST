import math

# 通信与充电相关参数（对齐Matlab代码参数）
B = 1 * 10**6  # 带宽 (Hz)，来自parameter_setting.m
P_tra = 3
Pk = 0.3  # 发射功率 (W)，来自parameter_setting.m
sigma_2 = -110  # 噪声功率 (dBm)，来自parameter_setting.m
beta0 = -60  # 参考路径损耗 (dB)，来自parameter_setting.m
alpha = 2  # 路径损耗指数，来自parameter_setting.m
Kc = 10  # 瑞利衰落参数，来自parameter_setting.m
delta_t = 0.5  # 时间间隔(s)，来自parameter_setting.m

class InputParameter:
    UAV_POWER = 1000000  # 无人机电量,单位(J)
    UAV_SPEED = 20
    # 地图参数设置
    AREA_LONG = 200     # 地图-长
    AREA_WIDE = 100     # 地图-宽
    SENSOR_POWER = 6000

    UAV_FLIGHT_HEIGHT = 100  # 无人机飞行高度,单位(m)

    def __init__(self):
        # 区域大小属性
        self.area_size = (self.AREA_LONG, self.AREA_WIDE)  # 添加一个元组表示区域大小

        # 基站参数
        self.base_station = (self.area_size[0] / 2, self.area_size[1] / 2, 10)  # 基站坐标(50,50),高度设置为固定10

        # 设备数量
        self.uav_num = 3  # 无人机数量
        self.sensor_num = 50  # 传感器数量



def getPathLoss(distance = 1):
    """计算路径损耗 (dB)，对应Matlab的getPathLoss函数"""
    # return beta0 - alpha * dec2dB(distance)\
    return beta0 / distance

def getAchievableRate(pathloss):
    """计算可达速率 (bps/Hz)，对应Matlab的getAchievableRate函数"""
    numerator = dB2dec(pathloss)
    denominator = dBm2dec(sigma_2)
    return math.log2(1 + Pk * numerator / denominator)

def computeDataTransmissionEnergy(distance = 1, data_size = 1024 * 1024):
    """
    计算数据传输能耗（基于Matlab中能量计算公式）
    :param distance: 与传感器节点的距离(m)
    :param data_size: 传输的数据量(bit)
    :return: 传输能耗(J)
    """
    # 计算路径损耗
    # pathloss = self.getPathLoss(distance)
    # 计算可达速率 (bps/Hz)
    pathloss = getPathLoss(distance)
    rate = getAchievableRate(pathloss)

    # 总传输速率 (bps) = 速率(bps/Hz) * 带宽(Hz)
    total_rate = rate * B
    # 传输时间 (s) = 数据量(bit) / 传输速率(bps)
    trans_time = data_size / total_rate
    # 传输能耗 (J) = 发射功率(W) * 传输时间(s)
    E_data = Pk * trans_time
    return E_data

def dec2dB(dec):
    """分贝转十进制，对应Matlab的dec2dB函数"""
    return 10 * math.log10(dec)

def dB2dec(dB):
    """十进制转分贝，对应Matlab的dB2dec函数"""
    return 10 **(dB / 10)

def dBm2dec(dBm):
    """dBm转十进制"""
    return 10 ** (dBm / 10) * (10 **(-3))


