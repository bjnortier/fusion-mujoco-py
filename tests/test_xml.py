# -*- coding: utf-8 -*-

from .context import fusion_mujoco_py

def test_empty():
    assert fusion_mujoco_py.to_xml('cheetah') == '<mujoco model="cheetah"><compiler angle="radian" coordinate="local" inertiafromgeom="true" /></mujoco>'
