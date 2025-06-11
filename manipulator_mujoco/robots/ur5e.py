import os
from manipulator_mujoco.robots.arm import Arm

_UR5E_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/ur5e/ur5e.xml',
)

_JOINTS = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)

# _EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class UR5e(Arm):
    def __init__(self, name: str = None):
        super().__init__(_UR5E_XML, _ATTACHMENT_SITE, _JOINTS, name)