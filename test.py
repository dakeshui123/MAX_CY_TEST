import math

# =========================
# 参数输入区（你可视场景调整）
# =========================

# 无人机参数
H = 50.0                        # UAV 悬停高度，单位：米
beta0 = 1e-3                    # 参考信道增益（1m 处）
eta = 0.5                       # RF-to-DC 转换效率 η'
P_tra = 10.0                    # UAV 充电发射功率，单位：瓦
Pt = 0.1                        # 传感器发射功率，单位：瓦
sigma2 = 1e-9                   # 接收端噪声功率，单位：瓦
gamma = beta0 / sigma2          # 信道参数组合
C = 20000.0                     # UAV 电量上限，单位：焦耳
P_hov = 200.0                   # 悬停推进功率，单位：瓦
P_mov = 150.0                   # 巡航推进功率（假设常数），单位：瓦
V_MR = 10.0                     # 巡航速度，单位：m/s
L_max = 1000.0                  # 航线总距离，单位：米

# 节点列表，每个节点是字典，包含位置 (x, y)、待传数据量 Li（bits）、所需补能 e_i（焦耳）
nodes = [
    {'w': (0, 0),     'L': 1e6, 'e': 50.0},
    {'w': (100, 0),   'L': 2e6, 'e': 100.0},
    {'w': (0, 100),   'L': 1.5e6, 'e': 80.0},
]

# ========================================
# 计算函数区：信道增益、充电时间、传输时间等
# ========================================

def channel_gain(q, w, H, beta0):
    dx = q[0] - w[0]
    dy = q[1] - w[1]
    d2 = dx*dx + dy*dy + H*H
    return beta0 / d2

def wpt_time(e_i, h, eta, P_tra):
    if h <= 0:
        return float('inf')
    return e_i / (eta * P_tra * h)

def wit_time(L_i, Pt, gamma, H):
    h = gamma / (H*H)
    rate = math.log2(1 + Pt * h)
    if rate <= 0:
        return float('inf')
    return L_i / rate

# ====================================
# 主计算逻辑：对每个节点循环计算时间
# ====================================

print("节点编号 | 收能源 (P_rec, W) | 充电时间 (s) | 上传时间 (s)")
t_total_wpt = 0.0
t_total_wit = 0.0
q_hover = (0, 0)  # UAV 悬停位置，假设与第一个节点对齐（可改为动态航迹）

for idx, nd in enumerate(nodes, start=1):
    w = nd['w']
    Li = nd['L']
    ei = nd['e']
    h = channel_gain(q_hover, w, H, beta0)
    P_rec = eta * P_tra * h
    t_wpt = wpt_time(ei, h, eta, P_tra)
    t_wit = wit_time(Li, Pt, gamma, H)
    t_total_wpt += t_wpt
    t_total_wit += t_wit
    print(f"{idx:^8d} | {P_rec:>12.6f} | {t_wpt:>11.2f} | {t_wit:>11.2f}")

# 总悬停时间
t_hover = t_total_wpt + t_total_wit

# 能量消耗对比
energy_cruise = (L_max / V_MR) * P_mov
energy_hover = t_hover * (P_hov + P_tra)
total_energy = energy_cruise + energy_hover

print("\n总悬停时间: {:.2f} s".format(t_hover))
print("巡航能耗: {:.2f} J".format(energy_cruise))
print("悬停能耗: {:.2f} J".format(energy_hover))
print("总能耗: {:.2f} J".format(total_energy))
print("是否满足 UAV 能量上限（C = {:.2f} J）？ {}".format(C, total_energy <= C))
