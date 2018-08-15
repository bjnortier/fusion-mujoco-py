#!/usr/bin/env python3
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math
import os
import argparse

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('file', metavar='file', type=str,
                    help='the Mujoco XML file')
args = parser.parse_args()

model = load_model_from_path(args.file)
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
while True:
#     t += 1
    # if sim.data.qpos[0] > math.pi/8:
    #     dir = -dir
    # if sim.data.qpos[0] < 0:
    #     dir = -dir
    # if dir == -1 and sim.data.qpos[0] < -math.pi/8:
    #     dir = 1
    # if dir == 1 and sim.data.qpos[0] > 0:
    #     dir = -1
    # sim.data.ctrl[0] = 10 * dir
    # print(dir, sim.data.qpos[0])
    sim.step()
    viewer.render()
