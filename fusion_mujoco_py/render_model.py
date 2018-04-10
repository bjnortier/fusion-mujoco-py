#!/usr/bin/env python3
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math
import os

model = load_model_from_path('/Users/bjnortier/fusion-mujoco-export/Basic v16.xml')
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
dir = -1
while True:
    t += 1
    # if sim.data.qpos[0] > math.pi/8:
    #     dir = -dir
    # if sim.data.qpos[0] < 0:
    #     dir = -dir
    if dir == -1 and sim.data.qpos[0] < -math.pi/8:
        dir = 1
    if dir == 1 and sim.data.qpos[0] > 0:
        dir = -1
    sim.data.ctrl[0] = 10 * dir
    print(dir, sim.data.qpos[0])
    sim.step()
    viewer.render()
