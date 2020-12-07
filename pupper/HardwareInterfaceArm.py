from pypot.robot import from_json


class HardwareInterfaceArm:
    def __init__(self, config="/home/pi/pypot.json"):
        self.robot = from_json(config)
        self.pos_rest = [0,-60,60,0]
        self.pos_erect = [0,0,0,0]
        self.gripper = 0

    def start(self):
        for m in self.robot.motors:
            m.compliant = False
            m.moving_speed = 50
            m.goal_position = 0

    def move(self, ms):
        for i in range(3):
            self.robot.motors[i].goal_position = ms[i]
        self.robot.motors[3].goal_position = self.gripper

    def gripper_open(self):
        self.gripper = -20

    def gripper_close(self):
        self.gripper = 20

    def gripper_neutral(self):
        self.gripper = 0