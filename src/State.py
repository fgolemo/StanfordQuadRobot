import numpy as np
from enum import Enum
import time

class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3

class ArmState(Enum):
    DEACTIVATED = -1
    RUNNING = 0

class GripperState(Enum):
    NEUTRAL = 0
    OPEN = 1
    CLOSED = 2

class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.16
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.behavior_state = BehaviorState.REST
        self.arm_state = ArmState.DEACTIVATED
        self.gripper_state = GripperState.NEUTRAL
        self.arm_joint_angles = np.zeros(4, dtype=np.float32)
        self.arm_x = 0
        self.arm_y = .155
        self.arm_z = .143
        self.last_ik = time.time()

        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
