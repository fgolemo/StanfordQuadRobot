from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np


class ArmIK:
    def __init__(self):
        self.chain = Chain(name='pupper_ergo', links=[
            OriginLink(),
            URDFLink(
                name="twistybase",
                translation_vector=[0, 0, .03],
                orientation=[0, 0, 0],
                rotation=[0, 0, 1],
            ),
            URDFLink(
                name="shoulder",
                translation_vector=[0, 0, .0285],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],
            ),
            URDFLink(
                name="elbow",
                translation_vector=[0, 0, .085],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],
            ),
            URDFLink(
                name="tip",
                translation_vector=[0, .155, 0],
                orientation=[1.57, 0, 0],
                rotation=[1, 0, 0],
            )
        ])

    @staticmethod
    def _ik2pupperarm(joints):
        return np.rad2deg([joints[1], -joints[2], -joints[3], 0.0])

    @staticmethod
    def _makeMoveCmd(degs):
        print(f"move([{','.join([str(x) for x in np.around(degs, 2)])}])")

    def pos2joints(self, target):
        joints = self.chain.inverse_kinematics(target)
        return self._ik2pupperarm(joints)
