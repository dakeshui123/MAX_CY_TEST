from __future__ import annotations
from queue import PriorityQueue
import numpy as np
import enum
import math
from utils import *

class NodeType(enum.Enum):
    BS = 0   # 基站
    SN = 1   # 传感器节点
    RN = 2   # 中继节点

class Node():
    """Node.
    """

    def __init__(self, position, _id, _type=NodeType.SN, is_active=True):
        self.position = position
        self.id = _id
        self.adj = list()
        self.is_active = is_active
        self.type = _type

    def __repr__(self):
        return str((self.id, self.position))

    def __str__(self):
        return str((self.id, self.position))



class Sensor(Node):
    """Sensor.
    """

    def __init__(self, position, battery_cap, _id, **kwargs):
        # 传感器最大容量
        self.battery_cap = battery_cap
        # 剩余能量
        self.cur_energy = battery_cap

        super(Sensor, self).__init__(position, _id,
                                     NodeType.SN, **kwargs)

    def get_state(self):
        return np.array([self.position.x,
                         self.position.y,
                         self.position.z,
                         self.battery_cap,
                         self.cur_energy,
                         self.consumedEnergy,
                         ],
                        dtype=np.float32)

    def reset(self):
        """reset.
        """
        self.cur_energy = self.battery_cap
        self.ecr = None
        self.activate()

    def deactivate(self):
        """deactivate.
        """
        self.is_active = False

    def activate(self):
        """activate.
        """
        self.is_active = True