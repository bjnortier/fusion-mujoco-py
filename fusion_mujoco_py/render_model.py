#!/usr/bin/env python3
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math
import os

model = load_model_from_path('/Users/bjnortier/fusion-mujoco-export/Basic v5.xml')
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
while True:
    t += 1
    sim.step()
    viewer.render()
