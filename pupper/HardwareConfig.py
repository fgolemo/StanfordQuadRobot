"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
    ##    [[ -0,   7,   2,   3], [ 17,  57, 46,  52], [-39, -35, -33, -64]]
    # [[ -0,  -4, -13,  -9],[ 51,  45,  37,  45],[-53, -47, -45, -46]]
    # [[ 14.,  -1., -11.,  -8.], [ 54.,  56.,  37.,  46.],[-53., -48., -45., -45.]]
    # [[ 13.,  -3., -13.,  -9.], [ 53.,  56.,  36.,  45.], [-53., -48., -45., -46.]]
    [[-1.0, -4.0, -14.0, -10.0], [45.0, 47.0, 43.0, 40.0], [-41.0, -43.0, -28.0, -22.0]]
)

PS4_COLOR = {"red": 0, "blue": 255, "green": 0}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 50, "green": 0}
PS4_RECORD_COLOR = {"red": 255, "blue": 0, "green": 0}
PS4_ARM_COLOR = {"red": 0, "blue": 0, "green": 255}
