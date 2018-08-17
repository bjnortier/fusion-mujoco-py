#!/usr/bin/env python3
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math
import os
import argparse

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('file',
                    metavar='file',
                    type=str,
                    help='the Mujoco XML file')
parser.add_argument('--nostep',
                    action='store_true',
                    help='step the simulation')
args = parser.parse_args()

model = load_model_from_path(args.file)
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
while True:
    if not args.nostep:
        sim.step()
    viewer.render()
