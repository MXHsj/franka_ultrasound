# ! /usr/bin/env python3
import os
import json
import numpy as np
from frankx import Affine, Robot, JointMotion, LinearRelativeMotion, PathMotion

robot = Robot("172.16.0.2")
# robot.set_default_behavior()  # ignore eef?
# robot.recover_from_errors()
robot.set_dynamic_rel(0.05)

# read F_T_EE from config file
# f = open(os.path.join(os.path.dirname(__file__), '../config/Clarius-probe-config.json'))
# EE_cfg = json.load(f)
# EE = (EE_cfg['transformation'])
# print('EE', EE)
# robot.set_EE(EE)

# Get the current pose
current_pose = robot.current_pose()
print(current_pose)
state = robot.read_once()
print('F_T_EE: ', state.F_T_EE)

# linear motion
# motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0))
# robot.move(motion)

# joint motion
# motion = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
# robot.move(motion)

# path motion
motion = PathMotion([
    Affine(0.5, 0.0, 0.35),
    Affine(0.5, 0.0, 0.24, -0.3),
    Affine(0.5, -0.2, 0.35),
], blend_max_distance=0.05)
robot.move(motion)
